---
name: revlib
description: REVLib 2026 API reference for SPARK MAX / SPARK Flex - configuration objects, closed-loop control, MAXMotion, native units (RPM!), and SparkSim simulation. Use when reading or writing any code that imports com.revrobotics, or when reviewing motor IO layers for REV hardware.
---

# REVLib 2026 (SPARK MAX / SPARK Flex)

Vendordep in this repo: **REVLib 2026.0.5** (`vendordeps/REVLib.json`).

Primary sources:
- API docs: https://codedocs.revrobotics.com/java/
- Guide: https://docs.revrobotics.com/revlib/
- Changelog: https://docs.revrobotics.com/revlib/install/changelog

## THE #1 REVLIB BUG: native units are RPM

This is the single most common defect in REV code, and it is invisible until the mechanism runs
60× too fast or too slow.

| API | Native unit |
|---|---|
| `RelativeEncoder.getPosition()` | **rotations** |
| `RelativeEncoder.getVelocity()` | **RPM** |
| `SparkClosedLoopController.setSetpoint(x, kVelocity)` | **RPM** |
| `SparkClosedLoopController.setSetpoint(x, kPosition)` | **rotations** |
| `SparkClosedLoopController.setSetpoint(x, kVoltage)` | volts |
| `SparkClosedLoopController.setSetpoint(x, kCurrent)` | amps |
| `SparkClosedLoopController.setSetpoint(x, kDutyCycle)` | −1..1 |
| `MAXMotionConfig.cruiseVelocity(x)` | **RPM** |
| `MAXMotionConfig.maxAcceleration(x)` | **RPM per second** |
| `SparkSim.iterate(velocity, vbus, dt)` | velocity in **RPM**, dt in **seconds** |

All of these are scaled by the configured conversion factors:

```java
config.encoder.positionConversionFactor(1.0);      // default: 1 -> rotations
config.encoder.velocityConversionFactor(1.0);      // default: 1 -> RPM
// To work in rotations/second everywhere:
config.encoder.velocityConversionFactor(1.0 / 60.0);
```

**Contrast with CTRE, which is rotations/second natively.** Any abstraction that presents one
`MotorIO` interface over both vendors must either normalize the units in the REV implementation
(`.in(RotationsPerSecond) * 60`) or *require and verify* `velocityConversionFactor == 1/60`.
Silently assuming the caller set it is a 60× landmine.

Recommended: convert explicitly at the boundary and do not depend on caller config.

```java
// Writing a velocity setpoint:
controller.setSetpoint(velocity.in(RPM), ControlType.kVelocity, slot);
// Reading velocity:
inputs.velocity = RPM.of(encoder.getVelocity());
```
(`edu.wpi.first.units.Units.RPM` and `RotationsPerSecond` interconvert automatically.)

## What changed in 2026

| Change | Impact |
|---|---|
| `SparkClosedLoopController.setReference(...)` **deprecated** → `setSetpoint(...)` | Rename all call sites |
| `configure(config, ResetMode, PersistMode)` **deprecated** as of 2026 | Check current docs for the replacement signature |
| `ResetMode` / `PersistMode` moved to a **common** enum: `com.revrobotics.ResetMode`, `com.revrobotics.PersistMode` | No longer nested per-device |
| **SmartMotion removed** — replaced by MAXMotion | `kSmartMotion*` control types are gone |
| MAXMotion `maxVelocity` → **`cruiseVelocity`**; `allowedClosedLoopError` → **`allowedProfileError`** | Rename |
| Feedforward expanded: `kV` (replaces `kF`), `kA`, `kS`, `kG`, `kCos`, `kCosRatio` | Under `config.closedLoop.feedForward` |
| New signals: `getMAXMotionSetpointPosition()`, `getMAXMotionSetpointVelocity()`, `isAtSetpoint()`, `getSetpoint()`, `getSelectedSlot()` | |
| `SparkSim.iterate()` now simulates current limits + closed-loop control | |
| Automatic `clearFaults()` on construction **removed** | Call it yourself if you relied on it |
| `DetachedEncoder` class added (MAXSpline encoders) | |
| `StatusLogger` (revlog → wpilog / AdvantageScope) added | |

## Configuration

REVLib config is **declarative and destructive**: you build a config object and apply it.

```java
SparkMaxConfig cfg = new SparkMaxConfig();     // or SparkFlexConfig
cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false);
cfg.encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0 / 60.0);
cfg.closedLoop.p(0.1, ClosedLoopSlot.kSlot0).i(0).d(0);
cfg.closedLoop.feedForward.kV(0.0, ClosedLoopSlot.kSlot0).kS(0).kG(0).kA(0);
cfg.closedLoop.maxMotion.cruiseVelocity(4000).maxAcceleration(6000).allowedProfileError(0.1);

motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
```

**`ResetMode`**
- `kResetSafeParameters` — reset everything not in this config to factory default. Use **once**,
  at construction, so behavior is reproducible.
- `kNoResetSafeParameters` — merge; leaves untouched parameters alone. Use for **incremental**
  updates (changing just PID, just a current limit, just idle mode).

**`PersistMode`**
- `kPersistParameters` — **writes to flash.** Slow, and flash has a finite erase/write budget.
  Use only at construction.
- `kNoPersistParameters` — RAM only. Use for every runtime update.

> Calling `configure(..., kPersistParameters)` more than once per boot — e.g. once in a base class
> constructor and again in a subclass constructor — doubles the flash writes for no benefit.

### `configure()` blocks

`configure()` is a **synchronous CAN transaction**. Calling it from `periodic()` (to flip idle
mode, retune PID, or change MAXMotion limits) will cause loop overruns. Queue it on a background
thread, or accept the cost knowingly and document it.

Unlike Phoenix 6's `DynamicMotionMagic*`, REVLib has **no** way to pass profile limits inside a
setpoint request — changing MAXMotion cruise velocity/acceleration *requires* a `configure()`
call. Any cross-vendor `runPosition(pos, slot, vel, accel)` API therefore has an unavoidable cost
asymmetry that should be documented.

## Closed-loop control

```java
SparkClosedLoopController c = motor.getClosedLoopController();
c.setSetpoint(value, ControlType, ClosedLoopSlot, arbFeedforward, ArbFFUnits);

ControlType.kDutyCycle | kVelocity | kVoltage | kPosition | kCurrent
         | kMAXMotionPositionControl | kMAXMotionVelocityControl
```
`ControlType` lives at `com.revrobotics.spark.SparkBase.ControlType`
(alias of `SparkLowLevel.ControlType`).

Telemetry: `getControlType()`, `getSetpoint()`, `getMAXMotionSetpointPosition()`,
`getMAXMotionSetpointVelocity()`, `isAtSetpoint()`, `getSelectedSlot()`, `getIAccum()`.

There are **4 slots** (`kSlot0..kSlot3`) — one more than Phoenix 6, which has only Slot0–2.

## Current: REV has no torque current

| Method | Meaning |
|---|---|
| `motor.getOutputCurrent()` | **stator / output** current (what the motor draws) |
| `config.smartCurrentLimit(int amps)` | limits **stator** current |
| — | REVLib exposes **no supply current** and **no torque current** signal |

So `getOutputCurrent()` is *not* the same quantity as Phoenix's `getSupplyCurrent()`. In a
cross-vendor IO layer, logging one into a field named `supplyCurrent` makes the two vendors'
logs non-comparable. Either rename the field to `outputCurrent`/`statorCurrent`, or document the
divergence loudly.

For an unavailable signal, log a **sentinel** (`Amps.of(Double.NaN)` or `Amps.zero()`) — never
`null`, which breaks struct/AdvantageKit logging.

## Error handling

```java
REVLibError err = motor.configure(...);       // every call returns a status
motor.getLastError();                          // status of the last getter
```
Getters return a plain `double` and report failure out-of-band via `getLastError()`, so the
`ifOk(spark, supplier, consumer)` / `tryUntilOk(spark, attempts, command)` helper pattern
(popularized by AdvantageKit templates) is the idiomatic wrapper. Apply it **consistently** —
a single unguarded `configure()` among guarded ones is a silent-failure path.

## Simulation — `SparkSim`

```java
SparkSim sim = new SparkMaxSim(sparkMax, dcMotor);   // or SparkFlexSim(sparkFlex, dcMotor)
sim.enable();                                        // or sim.useDriverStationEnable()

// every simulationPeriodic:
sim.setBusVoltage(RobotController.getBatteryVoltage());
sim.iterate(velocityRPM, busVoltage, dtSeconds);     // velocity in RPM (post-conversion-factor)

sim.getPosition(); sim.getVelocity(); sim.getAppliedOutput(); sim.getMotorCurrent();
sim.getSetpoint(); sim.getClosedLoopSlot();
sim.setPosition(double); sim.setVelocity(double);    // only if NOT calling iterate()
sim.getRelativeEncoderSim(); sim.getAbsoluteEncoderSim(); sim.getFaultManager();
```

Rules that bite:
- `iterate()` **drives** the sim; `setPosition`/`setVelocity` are for when you are *not* iterating.
  Mixing them fights the model.
- The `dt` you pass must be a real per-loop delta. Seeding `lastTime = 0` and computing
  `now - lastTime` on the first loop yields the **entire uptime** as one timestep and launches the
  simulated mechanism to infinity. Initialize `lastTime` in the constructor.
- `getSetpoint()` returns whatever setpoint was last sent, in whatever unit that control mode
  uses. It is **not** mode-tagged — pair it with `getControlType()` before interpreting.
- Followers need their own `SparkSim`; they are not driven by the leader's sim.

## Review checklist for REVLib code

- [ ] Every RPM↔RPS boundary converted explicitly (encoder velocity, velocity setpoints,
      MAXMotion cruise velocity/accel, `SparkSim.iterate`)
- [ ] `setSetpoint()` everywhere, no leftover `setReference()`
- [ ] `kPersistParameters` + `kResetSafeParameters` only at construction, exactly once
- [ ] Runtime updates use `kNoReset` + `kNoPersist`
- [ ] `configure()` not called from a periodic loop, or the cost is documented
- [ ] All `configure()` calls wrapped in `tryUntilOk`, all getters in `ifOk`
- [ ] Unsupported signals (torque current, supply current, temperature-in-sim) logged as a
      sentinel, never `null`
- [ ] `SparkSim.iterate` dt initialized correctly; followers have their own sim objects
- [ ] MAXMotion limits changed via `configure()` — cost understood vs CTRE's dynamic requests
