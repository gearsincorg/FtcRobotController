# BIOBUZZ FTC Robot Controller

Fork of the FIRST `FtcRobotController` SDK (`origin` = gearsincorg/FtcRobotController, `upstream` = FIRST-Tech-Challenge). Team code lives in `TeamCode/`; `FtcRobotController/` is the stock SDK module and should not be edited. Working branch: `BIOBUZZ-Master`.

## Stack
- **Pedro Pathing 3.0** for driving/localization. Hardware config is in `pedro/Constants.java` (`Constants.create(hardwareMap)` returns the `Follower`; `create(hardwareMap, maxPathSpeed)` caps path speed in in/s, used by auto via `AUTO_MAX_SPEED`): mecanum drive, OctoQuad localizer, Foresight. Tuners are in `pedro/procedures/` and `pedro/Tuning.java`.
- **SolversLib** command framework for opmodes, subsystems and button bindings (WPILib style). Replaced Ivy.
- Opmodes extend `CommandOpMode`; subsystems extend `SubsystemBase` and expose `...Command()` methods.

## Two chassis (prototype and production)
- The drive geometry differs, so `Constants` has a localizer + Foresight set per chassis; motor and pod directions are per chassis (gearing differs); motor names and OctoQuad ports are shared.
- The chassis is picked at runtime from the active robot configuration name: `res/xml/prototype.xml` or `res/xml/production.xml` (deployed with the app; select it on the Driver Station). Any other name throws at init. `Constants.chassis()` returns it; teleop shows it in telemetry.
- Keep the two XML configs in step when wiring changes. Tuners use the detected chassis, so paste tuning results into that chassis's block. Use `Constants.chassis()` for any other per-robot differences, but keep shared code shared.

## Layout (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode`)
- `GFORCE_Auto`, `GFORCE_Teleop`: competition opmodes (group "Competition"). Teleop is field-centric with heading lock; buttons are bound in `bindButtons()`.
- `Subsystems/`: one class per mechanism:
  - `IntakeSubsystem`: collect and transfer. Intake motor (`intake`), feeder motor (`feeder`, moves the whole column), gate servo (`gate`), four digital ball sensors (`slot1` nearest the intake .. `slot4` at the gate, active-low). Mode (IDLE, INTAKING, REJECTING, SHOOTING [WAIT_READY/FEED/CLEARING], EJECTING, UNJAMMING, MANUAL) is separate from the debounced ball count. A shot = slot4 clearing while feeding through the open gate. Full → stops and spits a 5th ball. `RAPID_FIRE` picks continuous vs wait-for-ready-each-ball. `TIMED_SHOTS = true` (temporary) ignores the sensors when shooting: every shoot request fires one ball (wait for shooter ready, feed `TIMED_FEED_MS`), and the shooter / point-at-goal don't switch off on "empty". Takes a `BooleanSupplier` for shooter-ready. The intake and feeder motors are real and must be in the config. The gate isn't fitted yet, so it's optional (skipped with a telemetry warning if missing). `SIMULATE_SENSORS = true` takes the ball count from `SimulatedSlots` (a column model) until the slot sensors are in use; telemetry warns while it's on.
  - `RobotCommands` (teamcode root): multi-subsystem commands for teleop and auto: `bindAutoAim` (track when full, idle when empty), `shootAll`, `shootOne`, `collectWhile(path)`.
  - Teleop controls: hold LB intake, A shoot all, RT shoot one per press, hold B eject; Right Stick press toggles point-at-goal and the shooter with it (the chassis turns to aim the shooter; on by itself when full, off when empty; twisting the right stick overrides); D-pad turns to a Pedro heading (up 90, left 180, down 270, right 0) and holds it until the stick is twisted (cancels point-at-goal); Share/Options select blue/red alliance; rumble on full / volley done / jam; gamepad LED shows count. Gamepad 2 sticks + X are a manual intake fallback (and D-pad up/down adds/removes simulated balls). `Util/IntakeTestOpMode` has raw motor/gate control (finding gate positions) and the state machine with RB as "shooter ready".
  - `VisionSubsystem`: webcam AprilTag cluster tracking.
  - `GimbalSubsystem`: two-servo gimbal (`pitch`, `roll`), positions -1..+1; `Util/GimbalOpMode` follows the right stick.
  - `ShooterSubsystem`: turret (motor + encoder), hood (servo) and one flywheel. It aims at the hive-Cell from Pedro odometry and leads the target to shoot on the move. Speed and hood come from `SHOT_TABLE` (distance → rpm, hood, flight time); tune it with `Util/ShooterTuningOpMode`. The turret must be centered at init. `TURRET_ENABLED = false` for now: the turret is held at 0 and the chassis aims instead (`getAimError()`), and `isReady()` checks the robot heading. Hive-Cell targets are set directly in Pedro coordinates, as observed on the field: red audience (60, 60), red far (84, 60), blue audience (60, 84), blue far (84, 84). The shooter aims at the nearer cell of the selected alliance. (The published FTC-frame table didn't map cleanly onto Pedro, so don't convert from it.) The shot table is a placeholder until measured. `FULL_SPEED = true` (temporary) runs the flywheel at full power instead of the table RPM and counts it ready `SPINUP_MS` after starting. Intake and transfer are in `IntakeSubsystem`.
- `Util/`: helper classes (`ClusterTracking`, `ClusterDisplay`) and test opmodes (`BasicOpMode_shaneshot`).
- `AutoPaths.java`: generated paths (see below).

## Autonomous paths: `.pp` -> Java
- Draw paths in the Pedro visualizer and save the `.pp` file in `/pp`. Commit every `.pp` file.
- `tools/pp2java.py` turns each `pp/<Name>.pp` into a public method `<name>()` in `AutoPaths.java` that returns a `FollowPathCommand`. Each method also calls `follower.setPose(start)` (the path's start point), when the method is called, not when the command runs.
- The Gradle task `:TeamCode:generatePaths` runs the script before every build (hooked to `preBuild`), so never hand-edit anything below the "Do not change anything above this line" banner in `AutoPaths.java`; it is overwritten. Code above the banner is preserved.
- A bad `.pp` file fails the build with a message and leaves `AutoPaths.java` untouched. If Python isn't installed, the task warns and skips.
- Run it manually with `python tools/pp2java.py`.

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
3. Python 3 on the PATH (`python --version`). Needed by the `generatePaths` Gradle task; without it `AutoPaths.java` can silently go stale. Verify with `python tools/pp2java.py` (should list every `pp/*.pp` file and leave `git status` clean).
4. Do a build in Android Studio and confirm it succeeds and `git status` stays clean.
5. Claude conversation history and memory are local to the old PC and are not in the repo; this file is the continuity.
