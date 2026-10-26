package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;

public class Path_NearCenter {
    private PoseFactory poseFactory;
    private BiobuzzRobot robot;

    public Path_NearCenter(BiobuzzRobot robot) {
        this.robot = robot;
        poseFactory = PoseFactory.degrees();
    }

    public Command autoCommand() {

        SequentialCommandGroup auto = new SequentialCommandGroup();
        robot.drive.setPose(getStartPose());

        // Add paths and other subsystem commands here
        auto.addCommands(robot.shooter.startTrackingCommand(), robot.intake.shootAllCommand(),robot.shooter.idleCommand());
        auto.addCommands(new ParallelCommandGroup(path_start(), robot.intake.intakeUntilFullCommand()));  // does not complete ???
        auto.addCommands(path_2(), robot.shooter.startTrackingCommand(),robot.intake.shootAllCommand());

        return auto;
    }

    // =================================================
    // Note:  Do not change anything above this line.
    //        Create a public method below for each file in the pp folder.
    // =================================================

    // Generated from NearCenter_start.turt
    public Pose getStartPose() {
        return poseFactory.of(57.4639, 10, 180); // from start path in this name group
    }

    public FollowPathCommand path_start() {
        final Pose start = getStartPose();
        final Pose garden = poseFactory.of(12.1178, 10, 180);
        final Pose gardenControl1 = poseFactory.of(35.5823, 14.6543, 0);
        final Pose gardenControl2 = poseFactory.of(26.8975, 8, 0);

        Path path_startPath = Paths.path(
                curve(start, gardenControl1, gardenControl2, garden).linear(start, garden)
        );

        return new FollowPathCommand(robot.drive, path_startPath);
    }

    // Generated from NearCenter_2.turt
    public FollowPathCommand path_2() {
        final Pose start = poseFactory.of(8.8354, 9.1705, 180);
        final Pose transition = poseFactory.of(32.0629, 32.8878, 90);
        final Pose transitionControl1 = poseFactory.of(30.0712, 18.7686, 0);
        final Pose transitionControl2 = poseFactory.of(32.0629, 22.1437, 0);
        final Pose farFlower = poseFactory.of(48.4112, 133.256, 89.4328);
        final Pose farFlowerControl1 = poseFactory.of(32.0629, 54.0812, 0);
        final Pose farFlowerControl2 = poseFactory.of(28.2895, 125.8201, 0);
        final Pose farFlowerControl3 = poseFactory.of(48.4112, 102.4128, 0);

        Path path_2Path = Paths.path(
                curve(start, transitionControl1, transitionControl2, transition).linear(start, transition),
                curve(transition, farFlowerControl1, farFlowerControl2, farFlowerControl3, farFlower).tangent()
        );

        return new FollowPathCommand(robot.drive, path_2Path);
    }

    // Generated from NearCenter_3.turt
    public FollowPathCommand path_3() {
        final Pose start = poseFactory.of(50.09, 130.51, 90);
        final Pose nearFlower = poseFactory.of(13.9818, 47.8964, 90);
        final Pose nearFlowerControl1 = poseFactory.of(21.4885, 123.7156, 0);
        final Pose nearFlowerControl2 = poseFactory.of(44.759, 40.7483, 0);

        Path path_3Path = Paths.path(
                curve(start, nearFlowerControl1, nearFlowerControl2, nearFlower).linear(start, nearFlower)
        );

        return new FollowPathCommand(robot.drive, path_3Path);
    }

    // Generated from NearCenter_4.turt
    public FollowPathCommand path_4() {
        final Pose start = poseFactory.of(14.45, 46.96, 90);
        final Pose nearShoot = poseFactory.of(55.12, 6.39, 90);
        final Pose nearShootControl1 = poseFactory.of(14.45, 6.39, 0);
        final Pose nearShootControl2 = poseFactory.of(55.12, 26.069, 0);

        Path path_4Path = Paths.path(
                curve(start, nearShootControl1, nearShootControl2, nearShoot).linear(start, nearShoot)
        );

        return new FollowPathCommand(robot.drive, path_4Path);
    }
}
