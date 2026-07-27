---
name: wpilib-frc
description: WPILib 2026 reference for FRC Java - the units library (Measure/Angle/AngularVelocity), physics simulation (DCMotorSim, FlywheelSim, ElevatorSim, SingleJointedArmSim), Alerts, RobotController/RoboRioSim timing, and the AdvantageKit IO-layer pattern with @AutoLog. Use when reading or writing WPILib code, designing an IO/hardware-abstraction layer, or reviewing simulation and logging code.
---

# WPILib 2026 (FRC Java)

WPILib **2026** via GradleRIO `2026.2.1`; AdvantageKit **26.0.2**. Java 17.

Primary sources:
- Docs: https://docs.wpilib.org/en/stable/
- Java API: https://github.wpilib.org/allwpilib/docs/release/java/
- AdvantageKit: https://docs.advantagekit.org/

## Units library (`edu.wpi.first.units`)

Immutable `Measure` objects. Types used in motor code:

| Type | Common units |
|---|---|
| `Angle` | `Rotations`/`Rotation`, `Radians`, `Degrees` |
| `AngularVelocity` | `RotationsPerSecond`, `RadiansPerSecond`, `RPM` |
| `AngularAcceleration` | `RotationsPerSecondPerSecond`, `RadiansPerSecondPerSecond` |
| `Voltage` | `Volts` | 
| `Current` | `Amps` |
| `Temperature` | `Celsius`, `Fahrenheit` |
| `Time` | `Seconds`, `Milliseconds` |
| `Distance` | `Meters`, `Inches` |

```java
Angle a = Rotations.of(2.5);
double rot = a.in(Rotations);
a.plus(b); a.minus(b); a.times(3.0); a.div(2.0);
a.lte(b); a.gte(b); a.isNear(b, 0.05);
Rotations.zero();
```

**Style notes that matter for a student-facing library:**
- `Rotation` is a *legal alias* for `Rotations` (likewise `Revolution`/`Revolutions`,
  `RPM`/`RotationsPerMinute`). Mixing `Rotation` and `Rotations` across files compiles fine but
  reads as if they were different things. **Pick one and use it everywhere.**
- `.times(double)` on an `Angle` returns an `Angle` — gear-ratio math is safe, but the units type
  system will **not** catch a rotor-vs-mechanism mixup. Only naming and docs can. Name parameters
  `rotorVelocity` / `mechanismPosition`, never bare `position`.
- Prefer `RotationsPerSecond.zero()` over `AngularVelocity.ofBaseUnits(0, RotationsPerSecond)`.

## Physics simulation

```java
DCMotor gearbox = DCMotor.getKrakenX60Foc(1);   // .getNEO(1), .getNeoVortex(1), .getKrakenX60(1)

var sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, jKgMetersSquared, gearing),
                         gearbox);
var fly = new FlywheelSim(plant, gearbox);
var arm = new SingleJointedArmSim(gearbox, gearing, moi, lenM, minAngle, maxAngle, gravity, start);
var ele = new ElevatorSim(gearbox, gearing, massKg, drumRadiusM, minH, maxH, gravity, startH);

sim.setInputVoltage(volts);
sim.update(dtSeconds);
sim.getAngularVelocityRadPerSec(); sim.getAngularPositionRad(); sim.getCurrentDrawAmps();
```

**Sim loop timing — the classic bug:**

```java
private Time lastTime = RobotController.getMeasureTime();   // INITIALIZE IN THE CONSTRUCTOR
public void periodic() {
    Time now = RobotController.getMeasureTime();
    double dt = now.minus(lastTime).in(Seconds);
    lastTime = now;
    ...
}
```
If `lastTime` starts at `Seconds.zero()`, the **first** `dt` is the entire process uptime
(often 5–30 s). One `update(20.0)` call sends the simulated mechanism to its limit instantly, and
it looks like a tuning problem rather than a timing bug. Alternatively just use the constant
`0.02` / `TimedRobot.kDefaultPeriod` — simpler and more predictable for students.

**Battery sim:**
```java
RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));
```
Then `RobotController.getBatteryVoltage()` reflects sag. Feed the *same* value into the vendor sim
state (`TalonFXSimState.setSupplyVoltage` / `SparkSim.setBusVoltage`) so both agree.

## Alerts

```java
Alert a = new Alert("Elevator motor disconnected!", AlertType.kError);   // kError/kWarning/kInfo
a.set(true);
```
Pair with a `Debouncer` so a single dropped CAN frame doesn't spam the dashboard:
```java
alert.set(debouncer.calculate(!connected));
```

## AdvantageKit IO-layer pattern

```java
public interface WidgetIO {
    @AutoLog class WidgetIOInputs {
        public boolean connected = false;
        public Angle position = Rotations.zero();
    }
    default void updateInputs(WidgetIOInputs inputs) {}
    default void setVoltage(Voltage v) {}
}
```
The annotation processor generates `WidgetIOInputsAutoLogged` with `toLog(LogTable)` /
`fromLog(LogTable)` / `clone()`.

### Never assign `null` to an `@AutoLog` field

This is worth stating explicitly because the failure is quiet:

- `LogTable.put(key, value)` does `if (value == null) return;` — the field is **silently omitted
  from the log**. AdvantageScope shows nothing; there is no warning.
- `LogTable.get(key, defaultValue)` calls `defaultValue.baseUnitMagnitude()` and
  `defaultValue.unit()` with **no null check** → **NPE during log replay**, the exact scenario
  AdvantageKit exists to support.
- Downstream getters hand `null` to user code, so every consumer needs a null guard. A `!= null`
  check appearing in mechanism code is a symptom that some IO implementation is doing this.

For a signal a given vendor genuinely cannot provide, log a **sentinel**:
```java
inputs.torqueCurrent = Amps.of(Double.NaN);   // "unsupported" — plots as a gap
inputs.torqueCurrent = Amps.zero();           // acceptable if NaN confuses students
```
Do the same for enums — use an `UNKNOWN`/`UNSUPPORTED` constant rather than `null`.

### Replay determinism

`updateInputs` must record **everything** the subsystem later reads. Anything derived from live
hardware outside `inputs` breaks replay. Corollary: two implementations of the same IO interface
should populate the *same set* of fields with the *same units and semantics* — otherwise the
interface is a lie and logs are not comparable across implementations.

## Designing an IO interface (relevant to hardware-abstraction libraries)

- **`default {}` methods are a footgun.** An interface full of empty defaults means a subclass
  that forgets to override a method compiles cleanly and silently does nothing at runtime. For
  operations every implementation must support, declare them **abstract** so the compiler catches
  the gap. Reserve `default` for genuinely optional behavior.
- Document the **frame of reference** (rotor vs mechanism) and the **units** of every parameter in
  the interface javadoc — that is the only contract the implementations share.
- Keep the real and sim implementations' `updateInputs` in sync. If sim overrides `updateInputs`
  wholesale by copy-paste, the two drift. Factor shared interpretation into a `protected` helper
  on the base class and have both call it.
- If a capability doesn't exist on one vendor, make that explicit (throw
  `UnsupportedOperationException`, or expose a `boolean supportsX()`), rather than inheriting a
  silent no-op.

## Review checklist

- [ ] No `null` assigned to any `@AutoLog` field
- [ ] Sim `dt` computed from a `lastTime` initialized in the constructor (or a fixed 0.02)
- [ ] `Rotation`/`Rotations` (and `RPM`/`RotationsPerSecond`) used consistently
- [ ] Rotor-side vs mechanism-side quantities named and documented distinctly
- [ ] Alerts debounced
- [ ] Interface methods that must be implemented are abstract, not empty defaults
- [ ] Real and sim implementations populate identical fields with identical units
- [ ] Javadoc `@param` names match the actual method signature
