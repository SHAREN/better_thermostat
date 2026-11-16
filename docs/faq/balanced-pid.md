---
title: Balanced PID for smooth valve control
description: Opt-in steady heating-power control for direct valves and 0–100% climate proxies.
slug: faq/balanced-pid
---

## When to use

A conventional proportional-heavy PID can repeatedly drive a heating valve
fully open and shut, even when the room temperature stays in a narrow range.
For slow emitters and mixing valves, enable **Balanced PID (experimental)**
in the TRV's advanced configuration while keeping **PID Controller** selected.
This is an explicit per-TRV option, **off by default**.

The mode only activates when the device supports **direct valve control** or
when a target-temperature-based climate proxy has the **0–100** setpoint range
that represents a heating-power percentage. Ordinary 5–30 °C thermostats
never receive a percentage as a temperature setpoint.

## How it works

- Starts with a bounded baseline estimate from learned heat loss and heating
  power. Since heating_power is learned from heating cycles at an unknown
  actuator position, the estimate is an initial prior, **not a calibrated
  fraction of 100% opening**.
- Maintains separate persistent integral correction to learn the real
  equilibrium position. Traditional PID gains and history remain untouched.
- Uses a conservative proportional correction, damped room-temperature slope,
  and slew limit of about **1.5 percentage points per minute**.
- **Immediately closes** if the room reaches 0.25 °C above the target or
  the target is lowered below the current temperature; coastdown prevents
  reopening until the room cools to within 0.03 °C of target.
- Still obeys higher-priority guards (HVAC off, open contacts, missing room
  temperature, and any valve maximum). Missing temperature yields zero.
- Does not change physical servo firmware, watchdogs or valve safety logic.

## Validation, deployment and rollback

Unit tests exercise the actual controller, changing target, sensor loss,
percent-mapped Vaillant proxy, restoring state, hard shutdown and a synthetic
thermal plant. These tests do **not** replace field measurements; the actuator
may be nonlinear and the weather/load may change.

After deploying a version that supports the option, enable it for one
thermostat, watch actual room temperature, opening, overshoot and physical
heat response, then adjust only if stable. To revert immediately, disable the
**Balanced PID** checkbox (classic PID remains the default), or use the
previous custom-component revision.

The mode does not guarantee an absolutely fixed valve position: weather,
occupancy and changes to the target temperature necessarily change the
required heat input.
