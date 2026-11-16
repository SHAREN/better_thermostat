"""Safety and simulated thermal-plant tests for opt-in balanced PID."""

from __future__ import annotations

from unittest.mock import patch

import pytest

from custom_components.better_thermostat.utils.calibration.pid import (
    PIDParams,
    PIDState,
    compute_pid,
)


def cycle(
    state: PIDState,
    *,
    now: float = 1300,
    target: float = 22.5,
    measured: float = 22.5,
    slope: float = 0.0,
    max_opening: float = 100,
    heat_loss: float = 0.003,
    heat_power: float = 0.017,
) -> tuple[float, dict]:
    """Run the real controller without wall-clock timing."""
    with patch(
        "custom_components.better_thermostat.utils.calibration.pid.monotonic",
        return_value=now,
    ):
        pct, debug, _ = compute_pid(
            PIDParams(balance_enabled=True),
            target,
            measured,
            measured,
            slope,
            "living_room",
            inp_current_temp_ema_C=measured,
            max_opening_pct=max_opening,
            heating_power_C_min=heat_power,
            heat_loss_C_min=heat_loss,
            state=state,
        )
    return pct, debug


def test_balance_holds_small_nonzero_valve_at_equilibrium():
    """A room at target should keep its heat-loss-compensating valve position."""
    state = PIDState(balance_base_pct=15, last_percent=15, pid_last_time=1000)
    pct, debug = cycle(state)
    assert 12 <= pct <= 18
    assert debug["mode"] == "pid_balance"
    assert 12 <= debug["base"] <= 18


def test_balance_limits_valve_acceleration():
    """An ordinary temperature deficit cannot produce an immediate 90% jump."""
    state = PIDState(balance_base_pct=15, last_percent=0, pid_last_time=1000)
    pct, _ = cycle(state, measured=21.0)
    assert pct <= 8  # 1.5 percentage points/minute over 5 minutes
    assert pct >= 1


def test_balance_closes_immediately_on_overshoot():
    """The overheating gate outranks the smooth-output rate limiter."""
    state = PIDState(
        balance_base_pct=15,
        balance_integral_pct=10,
        last_percent=85,
        pid_last_time=1000,
    )
    pct, debug = cycle(state, measured=22.81)
    assert pct == 0
    assert state.balance_integral_pct <= 0
    assert debug["u"] == 0


def test_lowering_setpoint_closes_immediately():
    """A user who lowers the target does not wait for the slew-rate ramp."""
    state = PIDState(
        balance_base_pct=15, last_percent=65, last_target_temp=22.5, pid_last_time=1000
    )
    pct, _ = cycle(state, target=22.0, measured=22.4)
    assert pct == 0


def test_setpoint_coastdown_stays_closed_until_near_new_target():
    """A lower setpoint cannot re-open the valve while the room is still hot."""
    state = PIDState(
        balance_base_pct=15, last_percent=65, last_target_temp=22.5, pid_last_time=1000
    )
    first, _ = cycle(state, now=1300, target=22.0, measured=22.4)
    assert first == 0
    assert state.balance_coastdown is True
    second, _ = cycle(state, now=1600, target=22.0, measured=22.18)
    assert second == 0
    assert state.balance_coastdown is True
    third, _ = cycle(state, now=1900, target=22.0, measured=22.01)
    assert state.balance_coastdown is False
    assert third >= 0


@pytest.mark.parametrize("limit", [0, 20, 45])
def test_respects_maximum_valve_opening(limit):
    state = PIDState(balance_base_pct=15, last_percent=10, pid_last_time=1000)
    pct, _ = cycle(state, target=24, measured=21, max_opening=limit)
    assert 0 <= pct <= limit


def test_missing_room_temperature_closes_safely():
    state = PIDState(last_percent=60, pid_last_time=1000)
    with patch(
        "custom_components.better_thermostat.utils.calibration.pid.monotonic",
        return_value=1300,
    ):
        pct, debug, _ = compute_pid(
            PIDParams(balance_enabled=True), 22.5, None, None, None, "room", state=state
        )
    assert pct == 0
    assert debug["error"] == "no_temps"


def test_prior_ignores_unusable_thermal_measurements():
    state = PIDState(last_percent=12, pid_last_time=1000)
    pct, debug = cycle(state, heat_loss=float("nan"), heat_power=0)
    assert pct > 0
    assert debug["base"] == 12.0


def test_balance_ignores_aggressively_auto_tuned_legacy_gains():
    state = PIDState(
        pid_kp=350,
        pid_ki=0.001,
        pid_kd=10000,
        balance_base_pct=15,
        last_percent=15,
        pid_last_time=1000,
    )
    pct, debug = cycle(state, measured=22.45)
    assert 10 <= pct <= 21
    assert debug["kp"] == 24
    assert state.pid_kp == 350  # preserved for disabling the balance switch


def test_balanced_control_on_slow_room_model():
    """Synthetic 15-minute emitter lag: hold target with modest valve motion.

    The model is deliberately not a replay of real hardware; it checks
    stability, safety and the presence of a nonzero equilibrium command.
    """
    state = PIDState(balance_base_pct=15, last_percent=15, pid_last_time=1000)
    target = 22.5
    room = 22.35
    emitter_fraction = 0.15
    previous = room
    valves = []
    readings = []

    for step in range(12 * 12):  # 12 hours at five-minute intervals
        now = 1300 + step * 300
        slope = (room - previous) / 5.0
        pct, debug = cycle(state, now=now, measured=room, target=target, slope=slope)
        valves.append(pct)
        readings.append(room)
        previous = room
        emitter_fraction += (pct / 100 - emitter_fraction) * (5.0 / 20.0)
        # At 15% average opening, heating exactly balances the 0.003 °C/min loss.
        room += 5.0 * (0.02 * emitter_fraction - 0.003)

    assert max(readings) < 22.85
    assert abs(sum(readings[-24:]) / 24 - target) < 0.12
    assert 3 < sum(valves[-24:]) / 24 < 32
    assert max(abs(a - b) for a, b in zip(valves, valves[1:])) <= 8


def test_pid_balance_option_survives_advanced_form_roundtrip() -> None:
    """The balance checkbox is explicit, with classic PID as the default."""
    from custom_components.better_thermostat.config_flow import (
        _build_advanced_fields,
        _normalize_advanced_submission,
    )
    from custom_components.better_thermostat.utils.const import CONF_PID_BALANCE_MODE

    normalized = _normalize_advanced_submission(
        {CONF_PID_BALANCE_MODE: True},
        default_calibration="target_temp_based",
        homematic=False,
        has_auto=False,
    )
    assert normalized[CONF_PID_BALANCE_MODE] is True
    fields = _build_advanced_fields(
        sources=[normalized],
        default_calibration="target_temp_based",
        homematic=False,
        has_auto=False,
    )
    field = next(k for k in fields if k.schema == CONF_PID_BALANCE_MODE)
    assert field.default() is True
    normalized_off = _normalize_advanced_submission(
        {}, default_calibration="target_temp_based", homematic=False, has_auto=False
    )
    assert normalized_off[CONF_PID_BALANCE_MODE] is False


def test_balance_state_survives_store_serialization() -> None:
    """Baseline, integral and coastdown latch must survive Home Assistant restarts."""
    from custom_components.better_thermostat.utils.state_manager import (
        RuntimeState,
        _deserialize,
        _serialize,
    )

    saved = RuntimeState()
    saved.pid["room"] = PIDState(
        balance_base_pct=16.5, balance_integral_pct=2.1, balance_coastdown=True
    )
    restored = _deserialize(_serialize(saved)).pid["room"]
    assert restored.balance_base_pct == 16.5
    assert restored.balance_integral_pct == 2.1
    assert restored.balance_coastdown is True


def test_classic_pid_unchanged_without_opt_in():
    """The default computation still executes the classic PID path."""
    state = PIDState(pid_kp=350, pid_ki=0.001, pid_kd=10000)
    with patch(
        "custom_components.better_thermostat.utils.calibration.pid.monotonic",
        return_value=1300,
    ):
        pct, debug, _ = compute_pid(
            PIDParams(auto_tune=False), 22.5, 22.4, 22.4, 0, "room", state=state
        )
    assert debug["mode"] == "pid"
    assert debug["base"] is None if "base" in debug else True
    assert pct >= 0
