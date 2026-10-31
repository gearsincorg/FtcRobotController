# BIOBUZZ FTC Robot Controller

Fork of the FIRST `FtcRobotController` SDK (`origin` = gearsincorg/FtcRobotController, `upstream` = FIRST-Tech-Challenge). Team code lives in `TeamCode/`; `FtcRobotController/` is the stock SDK module and should not be edited. Working branch: `BIOBUZZ-Master`.

## Stack
- **Pedro Pathing 3.0** for driving/localization. Hardware config is in `pedro/Constants.java` (`Constants.create(hardwareMap)` returns the `Follower`; `create(hardwareMap, maxPathSpeed)` caps path speed in in/s via Foresight's `maxVelocityConstraint`, used by auto via `AUTO_MAX_SPEED`; Foresight's own `maxPathSpeed` is a 0-1 fraction of top speed, not in/s): mecanum drive, OctoQuad localizer, Foresight. Tuners are in `pedro/procedures/` and `pedro/Tuning.java`.
- **SolversLib** command framework for opmodes, subsystems and button bindings (WPILib style). Replaced Ivy.
- Opmodes extend `CommandOpMode`; subsystems extend `SubsystemBase` and expose `...Command()` methods.

## Two chassis (prototype and production)
- The drive geometry differs, so `Constants` has a localizer + Foresight set per chassis; motor and pod directions are per chassis (gearing differs); motor names and OctoQuad ports are shared.
- The chassis is picked at runtime from the active robot configuration name: `res/xml/prototype.xml` or `res/xml/production.xml` (deployed with the app; select it on the Driver Station). Any other name throws at init. `Constants.chassis()` returns it; teleop shows it in telemetry.
- Keep the two XML configs in step when wiring changes. Tuners use the detected chassis, so paste tuning results into that chassis's block. Use `Constants.chassis()` for any other per-robot differences, but keep shared code shared.

## Layout (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode`)
- `GFORCE_Auto`, `GFORCE_Teleop`: competition opmodes (group "Competition"). Teleop is field-centric with heading lock; buttons are bound in `bindButtons()`.
- `Subsystems/`: one class per mechanism:
  - `IntakeSubsystem`: collect and transfer. Intake motor (`intake`), feeder/transfer motor (`feeder`, moves the whole column), gate servo (`gate`), four digital ball sensors (`slot1` nearest the intake .. `slot4` at the gate, active-low), one per ball position. The ball count is just the number of sensors seeing a ball (debounced `COUNT_DEBOUNCE_MS`): none = empty, all four = full. Nothing is inferred from balls passing a sensor. Five modes, picked every loop by priority (intake / feeder / gate): SHOOTING (shoot held and the shooter is up to speed: INTAKE / FEED / open), SPINUP (shoot held, shooter not up to speed, including after a shot slows it: both stopped), CLEAR_ALL (clear held: REJECT / EJECT), EJECTING (eject held: REJECT / HOLD), FULL (4 balls: REJECT / PACK), INTAKING (otherwise: INTAKE / FEED). The gate is open only in SHOOTING. While shoot is held (SPINUP or SHOOTING), clear and eject are ignored. Intaking is always on (from the first loop after START), so there's no intake button. Auto uses `shootAllCommand()` (ends when the count has read empty `SHOT_CLEAR_MS`, or `VOLLEY_TIMEOUT_MS` after feeding first starts) and `intakeUntilFullCommand()` (just waits for full); teleop uses `shootWhileHeldCommand()`, `ejectCommand()`, `clearAllCommand()`. These commands don't require the subsystem, so they can be held together. No manual mode. Takes a `BooleanSupplier` for shooter-ready. The intake and feeder motors are real and must be in the config. The gate isn't fitted yet, so it's optional (skipped if missing). `SIMULATE_SENSORS = true` takes the sensors from `SimulatedSlots` (a ball count packed against the gate: intake adds, feeding through the open gate shoots, full reverse ejects) until the slot sensors are in use; telemetry warns while it's on.
  - `RobotCommands` (teamcode root): multi-subsystem commands for teleop and auto: `bindAutoAim` (track when full, idle when empty), `shootAll` (volley), `shootWhileHeld`.
  - Teleop controls: the intake collects by itself; hold RB shoot, hold B eject (intake out, transfer holds), hold Y clear all (intake and transfer reversed); Right Stick press toggles point-at-goal and the shooter with it (the chassis turns to aim the shooter; on by itself when full, off when empty; twisting the right stick overrides); D-pad turns to a Pedro heading (up 90, left 180, down 270, right 0) and holds it until the stick is twisted (cancels point-at-goal); Share/Options select blue/red alliance; rumble on full / empty; gamepad LED shows count. Gamepad 2 D-pad up/down adds/removes simulated balls. `Util/IntakeTestOpMode` runs the intake state machine alone (A shoot, B eject, Y clear all, RB = "shooter ready").
  - `VisionSubsystem`: webcam AprilTag cluster tracking.
  - `GimbalSubsystem`: two-servo gimbal (`pitch`, `roll`), positions -1..+1; `Util/GimbalOpMode` follows the right stick.
  - `ShooterSubsystem`: turret (motor + encoder), hood (servo) and one flywheel. It aims at the hive-Cell from Pedro odometry and leads the target to shoot on the move. Speed and hood come from `SHOT_TABLE` (distance → rpm, hood, flight time); tune it with `Util/ShooterTuningOpMode`. The flywheel is a goBILDA 5203 1:1 (28 ticks/rev, 6000 rpm); `setVelocity` takes ticks/s, so convert with `rpmToTicksPerSec()`. `Util/FlywheelTestOpMode` measures free speed, suggested PIDF and step response. The turret must be centered at init. `TURRET_ENABLED = false` for now: the turret is held at 0 and the chassis aims instead (`getAimError()`), and `isReady()` checks the robot heading. Each alliance has two goals, in Pedro coordinates: red audience (60, 54.5), red far (84, 54.5), blue audience (60, 89.5), blue far (84, 89.5) (blue is red with y flipped about 72). The shooter aims at the nearer goal of the selected alliance. (The published FTC-frame table didn't map cleanly onto Pedro, so don't convert from it.) The shot table is a placeholder until measured. `FULL_SPEED = true` (temporary) runs the flywheel at full power instead of the table RPM and counts it ready `SPINUP_MS` after starting. Intake and transfer are in `IntakeSubsystem`.
- `Util/AutoConfig`: the auto setup menu shown during `GFORCE_Auto` init (gamepad 1 D-pad: up/down item, left/right change). Settings: alliance (goal side only; paths are not mirrored), start delay 0-15 s, start location (Audience 1/2, Far 1/2), cycles 0-2, park (None/Base). Saved to `GFORCE_auto.properties` on the hub; teleop starts on the saved alliance. `GFORCE_Auto.planFor()` maps each start location to its paths (null = not drawn yet, skipped with an init warning); the sequence is built in `preRun()` after START. To add paths: draw a `.turt` file, then fill in that location's `Plan`.
- `Util/`: helper classes (`ClusterTracking`, `ClusterDisplay`) and test opmodes (`BasicOpMode_shaneshot`).
- `paths/Path_<Group>.java`: generated autonomous paths (see below).

## Autonomous paths: `.turt` -> Java
- Draw paths in Turtle Tracer and save the `.turt` files in `/turt_files`. Commit every `.turt` file.
- Files are grouped by the text before the last underscore (`NearCenter_start.turt`, `NearCenter_2.turt` -> group `NearCenter`). Each group needs exactly one `_start` file; its start position becomes `getStartPose()`.
- `tools/turt2java.py` writes `paths/Path_<Group>.java` for each group: `path_start()`, then `path_<suffix>()` for the rest in natural order (2, 3, ... 10), each returning a `FollowPathCommand`. A new file is copied from the template `paths/Path_GroupName.java` (its `autoCommand()` runs every path in order). After that, everything above the "Do not change anything above this line" banner is yours to edit (e.g. `autoCommand()`); everything below is regenerated on every run.
- A bad group (e.g. no `_start` file) or `.turt` file prints an error and leaves every path file untouched.
- Run it manually with `python tools/turt2java.py`. The Gradle task `:TeamCode:generatePaths` does the same before every build (hooked to `preBuild`); a bad group fails the build. If Python isn't installed, the task warns and skips.
- `GFORCE_Auto` schedules `autoCommand()` on the first loop after START.

## Building
Build and deploy from Android Studio. The default shell Java may be 8, which is too old for the Gradle wrapper (needs 17+); use Android Studio's bundled JDK if building from a command line.

## Deploying to the robot (Wi-Fi ADB)
- The Gradle task `:TeamCode:adbConnect` (hooked to `preBuild`, so it runs on every build/Run) does `adb connect 192.168.43.1:5555` when the Control Hub is reachable. It never fails the build and skips quietly if you're not on the hub's Wi-Fi. `adb` is found via `sdk.dir` in `local.properties` or `ANDROID_HOME`.

## Conventions
- Keep opmodes thin; put mechanism logic in subsystems with command factories.
- Hardware names used so far: `front_left_drive`, `front_right_drive`, `back_left_drive`, `back_right_drive`, `octoquad`, `intake`, `feeder`, `gate`, `slot1`..`slot4`, `turret`, `shooter`, `hood`, `pitch`, `roll`, `Webcam 1`.

## Setting up a new PC
If the user says they are on a new machine, offer to run through this checklist (check each item with a command, don't assume):
1. `git clone` the repo and check out `BIOBUZZ-Master`. Confirm `git config user.name` / `user.email` are set.
2. Android Studio installed, with its bundled JDK (17+). Command-line `java -version` may still say 8; that's OK for Android Studio builds.
3. Python 3 on the PATH (`python --version`). Needed by the `generatePaths` Gradle task; without it `AutoPaths.java` can silently go stale. Verify with `python tools/turt2java.py` (should list every `turt_files/*.turt` file and leave `git status` clean).
4. Do a build in Android Studio and confirm it succeeds and `git status` stays clean.
5. Claude conversation history and memory are local to the old PC and are not in the repo; this file is the continuity.
