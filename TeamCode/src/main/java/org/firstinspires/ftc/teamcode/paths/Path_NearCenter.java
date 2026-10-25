package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot;

public class Path_NearCenter {
    private PoseFactory poseFactory;
    private Robot robot;

    public Path_NearCenter(Robot robot) {
        this.robot = robot;
        poseFactory = PoseFactory.degrees();
    }

    public Command autoCommand() {

        SequentialCommandGroup auto = new SequentialCommandGroup();
        robot.drive.setPose(getStartPose());

        // Add paths and other subsystem commands here
        auto.addCommands(robot.shooter.startTrackingCommand(), robot.intake.shootAllCommand(),robot.shooter.idleCommand());
        auto.addCommands(new ParallelRaceGroup(path_start(), robot.intake.intakeUntilFullCommand()));
        auto.addCommands(path_2(), robot.shooter.startTrackingCommand(),robot.intake.shootAllCommand());

        return auto;
    }

    // =================================================
    // Note:  Do not change anything above this line.
    //        Create a public method below for each file in the pp folder.
    // =================================================

    // Generated from NearCenter_start.turt
    public Pose getStartPose() {
        return poseFactory.of(55.1229, 6.3902, 180); // from start path in this name group
    }

    public FollowPathCommand path_start() {
        final Pose start = getStartPose();
        final Pose driveToShoot = poseFactory.of(10.245, 8, 180);
        final Pose driveToShootControl1 = poseFactory.of(34.6459, 13.0156, 0);
        final Pose driveToShootControl2 = poseFactory.of(39.539, 4.947, 0);

        Path path_startPath = Paths.path(
                curve(start, driveToShootControl1, driveToShootControl2, driveToShoot).linear(Math.toRadians(180), Math.toRadians(180))
        );

        return new FollowPathCommand(robot.drive, path_startPath);
    }

    // Generated from NearCenter_2.turt
    public FollowPathCommand path_2() {
        final Pose start = poseFactory.of(10.24, 8, 180);
        final Pose auto1Step2 = poseFactory.of(50.0886, 130.5078, 90);
        final Pose auto1Step2Control1 = poseFactory.of(56.9928, 33.0823, 0);
        final Pose auto1Step2Control2 = poseFactory.of(4.9555, 101.0356, 0);
        final Pose auto1Step2Control3 = poseFactory.of(51.5212, 107.3719, 0);

        Path path_2Path = Paths.path(
                curve(start, auto1Step2Control1, auto1Step2Control2, auto1Step2Control3, auto1Step2).linear(Math.toRadians(180), Math.toRadians(90))
        );

        return new FollowPathCommand(robot.drive, path_2Path);
    }

    // Generated from NearCenter_3.turt
    public FollowPathCommand path_3() {
        final Pose start = poseFactory.of(50.09, 130.51, 90);
        final Pose auto1StepD = poseFactory.of(14.45, 46.96, 180);
        final Pose auto1StepDControl1 = poseFactory.of(25, 125.5884, 0);
        final Pose auto1StepDControl2 = poseFactory.of(43.1203, 42.8552, 0);

        Path path_3Path = Paths.path(
                curve(start, auto1StepDControl1, auto1StepDControl2, auto1StepD).linear(Math.toRadians(90), Math.toRadians(180))
        );

        return new FollowPathCommand(robot.drive, path_3Path);
    }

    // Generated from NearCenter_7.turt
    public FollowPathCommand path_7() {
        final Pose start = poseFactory.of(14.45, 46.96, 180);
        final Pose auto1StepD = poseFactory.of(55.12, 6.39, 180);
        final Pose auto1StepDControl1 = poseFactory.of(14.45, 6.39, 0);
        final Pose auto1StepDControl2 = poseFactory.of(55.12, 26.069, 0);

        Path path_7Path = Paths.path(
                curve(start, auto1StepDControl1, auto1StepDControl2, auto1StepD).linear(Math.toRadians(180), Math.toRadians(180))
        );

        return new FollowPathCommand(robot.drive, path_7Path);
    }
}
