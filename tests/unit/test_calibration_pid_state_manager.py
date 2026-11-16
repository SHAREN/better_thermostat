"""Tests that PID calibration reads and writes state through the state manager."""

from unittest.mock import MagicMock

from custom_components.better_thermostat.calibration import _compute_pid_balance
from custom_components.better_thermostat.trv import Trv
from custom_components.better_thermostat.utils.calibration.pid import (
    PIDState,
    build_pid_key,
)


class _PidStateStub:
    """Minimal stand-in for the state manager's PID accessors."""

    def __init__(self) -> None:
        self.pid: dict[str, PIDState] = {}

    def get_pid(self, key: str) -> PIDState:
        """Return the stored state for ``key``, creating it on first access."""
        return self.pid.setdefault(key, PIDState())

    def set_pid(self, key: str, pid: PIDState) -> None:
        """Store ``pid`` under ``key``."""
        self.pid[key] = pid


def _make_bt(state_mgr: _PidStateStub) -> MagicMock:
    """Return a BetterThermostat mock wired for a single heating TRV."""
    bt = MagicMock()
    bt.device_name = "Test BT"
    bt.unique_id = "uid"
    bt.bt_target_temp = 22.0
    bt.cur_temp = 20.0
    bt.cur_temp_filtered = None
    bt.temp_slope = 0.0
    bt.window_open = False
    bt.bt_hvac_mode = "heat"
    bt.real_trvs = {
        "climate.trv": Trv.from_legacy_dict(
            "climate.trv",
            {
                "advanced": {},
                "current_temperature": 21.0,
                "min_temp": 5.0,
                "max_temp": 30.0,
            },
        )
    }
    bt.state_mgr = state_mgr
    return bt


def test_pid_balance_persists_learned_state_in_state_manager() -> None:
    """The computed PID state lands in the state manager under the PID key."""
    state_mgr = _PidStateStub()
    bt = _make_bt(state_mgr)

    _compute_pid_balance(bt, "climate.trv")

    key = build_pid_key(bt, "climate.trv")
    assert key in state_mgr.pid
    # error = |target - current| = |22.0 - 20.0|
    assert state_mgr.pid[key].last_abs_error == 2.0


def test_pid_balance_schedules_state_persistence() -> None:
    """A successful PID cycle schedules a save of the learned state."""
    state_mgr = _PidStateStub()
    bt = _make_bt(state_mgr)

    _compute_pid_balance(bt, "climate.trv")

    bt.schedule_save_state.assert_called()


def test_pid_balance_threads_the_same_state_across_calls() -> None:
    """Repeated calls keep accumulating on the state manager's state object."""
    state_mgr = _PidStateStub()
    bt = _make_bt(state_mgr)
    key = build_pid_key(bt, "climate.trv")

    _compute_pid_balance(bt, "climate.trv")
    first = state_mgr.pid[key]

    _compute_pid_balance(bt, "climate.trv")
    assert state_mgr.pid[key] is first
    assert first.previous_abs_error == 2.0


def test_balanced_pid_accepts_percent_mapped_vaillant_climate_proxy() -> None:
    """A 0..100 setpoint proxy is a servo, even without direct valve support."""
    state_mgr = _PidStateStub()
    bt = _make_bt(state_mgr)
    trv = bt.real_trvs["climate.trv"]
    trv.advanced.update({"calibration": "target_temp_based", "pid_balance_mode": True})
    trv.min_temp = 0
    trv.max_temp = 100
    trv.current_temperature = 0
    bt.cur_temp = 21.95
    bt.cur_temp_filtered = None
    bt.heating_power = 0.0062
    bt.heat_loss_rate = 0.00114

    percent, direct = _compute_pid_balance(bt, "climate.trv")
    assert percent is not None
    assert direct is False
    assert trv.calibration_balance["debug"]["mode"] == "pid_balance"
    assert trv.calibration_balance["debug"]["base"] is not None


def test_balanced_pid_will_not_apply_percent_to_ordinary_climate_setpoint() -> None:
    """Even if requested, a normal 5..30°C TRV stays on the classic PID."""
    state_mgr = _PidStateStub()
    bt = _make_bt(state_mgr)
    trv = bt.real_trvs["climate.trv"]
    trv.advanced.update({"calibration": "target_temp_based", "pid_balance_mode": True})

    _compute_pid_balance(bt, "climate.trv")
    assert trv.calibration_balance["debug"]["mode"] == "pid"


def test_pid_balance_ignores_the_room_sensor_filter_during_the_trv_fallback() -> None:
    """The filter still holds the lost sensor's readings; PID uses the TRV value."""
    state_mgr = _PidStateStub()
    bt = _make_bt(state_mgr)
    bt.cur_temp_filtered = 18.0
    bt.room_sensor_fallback = True

    _compute_pid_balance(bt, "climate.trv")

    key = build_pid_key(bt, "climate.trv")
    # error = |target - current| = |22.0 - 20.0|, not |22.0 - 18.0|
    assert state_mgr.pid[key].last_abs_error == 2.0
