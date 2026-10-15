# BIOBUZZ FTC Robot Controller

Fork of the FIRST `FtcRobotController` SDK (`origin` = gearsincorg/FtcRobotController, `upstream` = FIRST-Tech-Challenge). Team code lives in `TeamCode/`; `FtcRobotController/` is the stock SDK module and should not be edited. Working branch: `BIOBUZZ-Master`.

## Stack
- **Pedro Pathing 3.0** for driving/localization. Hardware config is in `pedro/Constants.java` (`Constants.create(hardwareMap)` returns the `Follower`): mecanum drive, OctoQuad localizer, Foresight. Tuners are in `pedro/procedures/` and `pedro/Tuning.java`.
- **SolversLib** command framework for opmodes, subsystems and button bindings (WPILib style). Replaced Ivy.
- Opmodes extend `CommandOpMode`; subsystems extend `SubsystemBase` and expose `...Command()` methods.

## Layout (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode`)
- `GFORCE_Auto`, `GFORCE_Teleop`: competition opmodes (group "Competition"). Teleop is field-centric with heading lock; buttons are bound in `bindButtons()`.
- `Subsystems/`: one class per mechanism (currently `FlowerSubsystem`, a two-motor collector with start/stop/reverse).
- `Util/`: helper classes (`ClusterTracking`, `ClusterDisplay`) and test opmodes (`BasicOpMode_shaneshot`).
- `AutoPaths.java`: generated paths (see below).

## Autonomous paths: `.pp` -> Java
- Draw paths in the Pedro visualizer and save the `.pp` file in `/pp`. Commit every `.pp` file.
- `tools/pp2java.py` turns each `pp/<Name>.pp` into a public method `<name>()` in `AutoPaths.java` that returns a `FollowPathCommand`.
- The Gradle task `:TeamCode:generatePaths` runs the script before every build (hooked to `preBuild`), so never hand-edit anything below the "Do not change anything above this line" banner in `AutoPaths.java`; it is overwritten. Code above the banner is preserved.
- A bad `.pp` file fails the build with a message and leaves `AutoPaths.java` untouched. If Python isn't installed, the task warns and skips.
- Run it manually with `python tools/pp2java.py`.

## Building
Build and deploy from Android Studio. The default shell Java may be 8, which is too old for the Gradle wrapper (needs 17+); use Android Studio's bundled JDK if building from a command line.

## Conventions
- Keep opmodes thin; put mechanism logic in subsystems with command factories.
- Hardware names used so far: `front_left_drive`, `front_right_drive`, `back_left_drive`, `back_right_drive`, `octoquad`, `flower1`, `flower2`.
