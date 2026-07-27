---
name: ctre-phoenix6
description: CTRE Phoenix 6 (2026) API reference for TalonFX/TalonFXS/CANcoder - control requests, status signals, configuration, followers, and simulation via TalonFXSimState. Use when reading or writing any code that imports com.ctre.phoenix6, or when reviewing motor/encoder IO layers for CTRE hardware.
---

# CTRE Phoenix 6 (2026)

Vendordep in this repo: **Phoenix6 26.3.0** (`vendordeps/Phoenix6-26.3.0.json`).

Primary sources:
- API docs: https://api.ctr-electronics.com/phoenix6/stable/java/
- Guide: https://v6.docs.ctr-electronics.com/en/latest/
- Yearly changelog: https://v6.docs.ctr-electronics.com/en/latest/docs/yearly-changes/yearly-changelog.html

## What changed in 2026 (check code against this)

| Change | Impact |
|---|---|
| `new TalonFX(int id, String canbus)` **deprecated**, removal in 2027 | Use `new TalonFX(int id, CANBus canbus)` |
| `optimizeBusUtilization()` default is now **4 Hz**, was 0 Hz | Pass an explicit frequency if you mean "off" |
| `Follower` takes `MotorAlignmentValue`, not a boolean | `new Follower(id, MotorAlignmentValue.Opposed \| .Aligned)` |
| Followers now follow **MotorVoltage** (not DutyCycle) when the leader is in voltage control, and mirror leader brake/coast | Follower behavior is more faithful; no code change needed |
| New `MotionMagicAtTarget` status signal | Prefer over hand-rolled "at goal" checks |
| New `GravityArmPositionOffset` config (±0.25 rot) | Offsets position for arm kG |
| `DynamicMotionMagicExpo*` control request added | |
| Simple gain scheduling for position PID based on closed-loop error | |
| `TalonFXSimState.setMotorType()` added (Kraken X44 sim) | |
| `BaseStatusSignal.isNear(target, tolerance)`, `StatusSignalCollection`, `List<BaseStatusSignal>` overloads | |
| `RefreshAll`/`WaitForAll` now report errors for **all** errored signals | |

## Status signals

`TalonFX` getters return `StatusSignal<T>`. **Get the units right — several are `Double`, not a unit type:**

```java
StatusSignal<Angle>            getPosition()          // mechanism rotations (SensorToMechanismRatio applied)
StatusSignal<Angle>            getRotorPosition()     // raw rotor rotations
StatusSignal<AngularVelocity>  getVelocity()
StatusSignal<AngularVelocity>  getRotorVelocity()
StatusSignal<Voltage>          getMotorVoltage()      // voltage APPLIED TO THE MOTOR
StatusSignal<Voltage>          getSupplyVoltage()     // battery/bus voltage into the controller
StatusSignal<Current>          getSupplyCurrent()
StatusSignal<Current>          getStatorCurrent()
StatusSignal<Current>          getTorqueCurrent()
StatusSignal<Temperature>      getDeviceTemp()
StatusSignal<Double>           getClosedLoopError()          // unitless: rot OR rot/s depending on mode
StatusSignal<Double>           getClosedLoopReference()      // ditto
StatusSignal<Double>           getClosedLoopReferenceSlope() // d(reference)/dt
```

**`getMotorVoltage()` != `getSupplyVoltage()`.** Confusing these is a common bug: supply
voltage sits near battery voltage (~12 V) at all times, so an "applied voltage" log fed from
`getSupplyVoltage()` looks plausible but is always wrong.

`getClosedLoopError()`/`getClosedLoopReference()` are **mode-dependent and unitless**. In position
modes they are rotations; in velocity modes rotations/sec. You must branch on the active control
request before attaching a unit. There is no signal that tells you which — inspect
`motor.getAppliedControl()`.

### Refresh pattern

```java
// Cache signals once in the constructor, then:
BaseStatusSignal.refreshAll(sig1, sig2, ...).isOK();     // returns StatusCode
BaseStatusSignal.setUpdateFrequencyForAll(100.0, sig1, sig2, ...);
motor.optimizeBusUtilization(0, 1.0);   // (optimizedFreqHz, timeoutSeconds)
```

Call `optimizeBusUtilization` on **every** device you construct, including followers — otherwise
followers keep broadcasting their full default signal set and eat CAN bandwidth.

## Control requests

Reuse request objects as fields; they are mutable builders. `withX()` mutates and returns `this`.

| Request | Notes |
|---|---|
| `CoastOut`, `StaticBrake`, `NeutralOut` | `NeutralOut` honors the configured neutral mode; the other two force it |
| `VoltageOut`, `DutyCycleOut` | `withEnableFOC(true)` requires a **Phoenix Pro license** |
| `TorqueCurrentFOC` | **Pro-licensed only.** `withMaxAbsDutyCycle()` caps top speed |
| `PositionVoltage`, `PositionTorqueCurrentFOC`, `PositionDutyCycle` | unprofiled |
| `MotionMagicVoltage`, `MotionMagicTorqueCurrentFOC`, `MotionMagicDutyCycle` | trapezoid/S-curve profiled |
| `MotionMagicExpo*` | exponential profile using kV/kA |
| `DynamicMotionMagic*` | cruise vel/accel/jerk **embedded in the request** — no reconfig needed |
| `VelocityVoltage`, `VelocityTorqueCurrentFOC` | unprofiled velocity |
| `MotionMagicVelocity*` | velocity ramped by Motion Magic acceleration |
| `Follower`, `StrictFollower`, `DifferentialFollower` | |

### FOC licensing — important for student-facing libraries

FOC and all `*TorqueCurrentFOC` requests require a **Phoenix Pro license per device**. On an
unlicensed device FOC requests fall back to non-FOC behavior, and `TorqueCurrentFOC` will not
behave as a torque request. A library whose *only* closed-loop path is `*TorqueCurrentFOC`
silently misbehaves on unlicensed hardware. Prefer `*Voltage` variants as the default, or make the
variant selectable.

### Changing Motion Magic limits per-call

Two options, and they are **not** equivalent:
1. `DynamicMotionMagicTorqueCurrentFOC` / `DynamicMotionMagicVoltage` — cruise velocity,
   acceleration, and jerk travel **inside the control request**. Takes effect immediately, no
   config write, no CAN round-trip. **Prefer this.**
2. `configurator.apply(new MotionMagicConfigs()...)` then `setControl(...)` — a config write.
   If the write is async, the first `setControl` runs against the **old** profile. If it is
   synchronous, it blocks the robot loop on a CAN transaction.

## Configuration

```java
var cfg = new TalonFXConfiguration();
cfg.Slot0.withKP(..).withKI(..).withKD(..).withKS(..).withKV(..).withKA(..).withKG(..);
motor.getConfigurator().apply(cfg);            // full config — resets unspecified fields to default
motor.getConfigurator().apply(cfg.Slot0);      // partial — only touches that config group
```

- **Only `Slot0`, `Slot1`, `Slot2` exist.** There is no Slot3. An abstraction exposing 4 slots
  cannot map the 4th onto a TalonFX.
- `apply(TalonFXConfiguration)` **overwrites every config group**. Applying a stale cached
  config object silently reverts unrelated settings.
- `TalonFXConfiguration` is a mutable object. If a caller shares one instance across several
  motors and the library stores the reference and mutates it, every motor is affected. **Defensive
  copy on construction.**
- Config writes are blocking CAN transactions (default 50 ms timeout). Never call them from a
  periodic loop; queue them onto a background thread.

### Feedback / gear ratios

```java
cfg.Feedback.SensorToMechanismRatio  // sensor rotations -> mechanism rotations
cfg.Feedback.RotorToSensorRatio      // rotor rotations -> sensor rotations
cfg.Feedback.FeedbackSensorSource    // RotorSensor (default), FusedCANcoder, SyncCANcoder, RemoteCANcoder
```

`RotorToSensorRatio` is **ignored when `FeedbackSensorSource == RotorSensor`** (the default). Code
that unconditionally multiplies by `RotorToSensorRatio * SensorToMechanismRatio` happens to be
correct only because the ratio defaults to 1.0.

`motor.setPosition(Angle)` sets the **mechanism** position — Phoenix divides by
`SensorToMechanismRatio` internally. Do not pre-multiply by the gear ratio.

## Simulation — `TalonFXSimState`

```java
TalonFXSimState sim = motor.getSimState();

sim.Orientation = ChassisReference.CounterClockwise_Positive;  // PUBLIC FIELD, default CCW+
sim.setSupplyVoltage(RobotController.getBatteryVoltage());     // min 4 V enforced
sim.setRawRotorPosition(Angle);        // ROTOR rotations
sim.addRotorPosition(Angle);
sim.setRotorVelocity(AngularVelocity); // ROTOR rot/s
sim.setRotorAcceleration(AngularAcceleration);
sim.setMotorType(TalonFXSimState.MotorType.…);  // 2026: Kraken X44 support

double  getMotorVoltage();      Voltage getMotorVoltageMeasure();
double  getSupplyCurrent();     Current getSupplyCurrentMeasure();
double  getTorqueCurrent();     Current getTorqueCurrentMeasure();
```

Rules that bite:
- **All setters take ROTOR-side values.** Convert from mechanism units by multiplying by the total
  gear ratio.
- **`Orientation` is not derived from the invert config.** SimState deliberately ignores
  `MotorOutput.Inverted`. Set `Orientation` when the *mechanical linkage* is reversed. A sim layer
  that never exposes `Orientation` cannot model an inverted gearbox.
- Set `setSupplyVoltage(...)` **before** refreshing status signals, or the signals you read this
  loop reflect last loop's supply voltage.
- Sim writes feed the normal status signals, so `motor.getPosition()` etc. work in sim — you do
  not need to read values back off the sim state, and mixing the two sources makes the real and
  sim IO layers disagree.

## Review checklist for Phoenix 6 code

- [ ] `getMotorVoltage()` used for applied voltage, not `getSupplyVoltage()`
- [ ] Closed-loop error/reference given units only after branching on the active control mode
- [ ] `optimizeBusUtilization` called on followers too
- [ ] `TalonFXConfiguration` defensively copied, not aliased from the caller
- [ ] No Slot3 assumptions
- [ ] Config writes off the main loop; Dynamic Motion Magic preferred over reconfigure-then-set
- [ ] Sim: `Orientation` set; setters given rotor-side units; supply voltage set before refresh
- [ ] `setPosition()` given mechanism units, not pre-multiplied
- [ ] FOC/Pro licensing considered if `*TorqueCurrentFOC` is the only closed-loop path
