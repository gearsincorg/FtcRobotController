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
        auto.addCommands(robot.shooter.startTrackingCommand(), robot.intake.shootAllCommand());
        auto.addCommands(new ParallelCommandGroup(path_start().withTimeout(2000), robot.intake.intakeUntilFullCommand()));
        auto.addCommands(path_2(),robot.intake.shootAllCommand());
        auto.addCommands(path_3(),robot.intake.intakeUntilFullCommand());
        auto.addCommands(path_4(),robot.intake.shootAllCommand(),robot.shooter.idleCommand());

        return auto;
    }

    // =================================================
    // Note:  Do not change anything above this line.
    //        Create a public method below for each file in the pp folder.
    // =================================================

    // Generated from NearCenter_start.turt
    public Pose getStartPose() {
        return poseFactory.of(32, 6, 180); // from start path in this name group
    }

    public FollowPathCommand path_start() {
        final Pose start = getStartPose();
        final Pose garden = poseFactory.of(7, 6, 180);
        final Pose gardenControl1 = poseFactory.of(15.7506, 6, 0);
        final Pose gardenControl2 = poseFactory.of(26.8975, 6, 0);

        Path path_startPath = Paths.path(
                curve(start, gardenControl1, gardenControl2, garden).linear(start, garden).with(maxVelocity(20))
        );

        return new FollowPathCommand(robot.drive, path_startPath);
    }

    // Generated from NearCenter_2.turt
    public FollowPathCommand path_2() {
        final Pose start = poseFactory.of(7, 6, 180);
        final Pose transition = poseFactory.of(36, 36, 90);
        final Pose transitionControl1 = poseFactory.of(30.0712, 18.7686, 0);
        final Pose transitionControl2 = poseFactory.of(36.2229, 20.1469, 0);
        final Pose point2 = poseFactory.of(48, 134.8865, 90);
        final Pose point2Control1 = poseFactory.of(36, 95.2591, 0);
        final Pose point2Control2 = poseFactory.of(33.3375, 127.1677, 0);
        final Pose point2Control3 = poseFactory.of(48, 105.9719, 0);

        Path path_2Path = Paths.path(
                curve(start, transitionControl1, transitionControl2, transition).linear(start, transition).with(maxVelocity(60)),
                curve(transition, point2Control1, point2Control2, point2Control3, point2).linear(transition, point2)
        );

        return new FollowPathCommand(robot.drive, path_2Path);
    }

    // Generated from NearCenter_3.turt
    public FollowPathCommand path_3() {
        final Pose start = poseFactory.of(48, 135, 90);
        final Pose nearFlower = poseFactory.of(11, 48, 180);
        final Pose nearFlowerControl1 = poseFactory.of(27.1462, 114.5635, 0);
        final Pose nearFlowerControl2 = poseFactory.of(48, 48, 0);

        Path path_3Path = Paths.path(
                curve(start, nearFlowerControl1, nearFlowerControl2, nearFlower).linear(start, nearFlower)
        );

        return new FollowPathCommand(robot.drive, path_3Path);
    }

    // Generated from NearCenter_4.turt
    public FollowPathCommand path_4() {
        final Pose start = poseFactory.of(11, 48, 180);
        final Pose nearShoot = poseFactory.of(32, 6, 180);
        final Pose nearShootControl1 = poseFactory.of(21.4363, 41.6746, 0);
        final Pose nearShootControl2 = poseFactory.of(32, 17.2497, 0);

        Path path_4Path = Paths.path(
                curve(start, nearShootControl1, nearShootControl2, nearShoot).linear(start, nearShoot)
        );

        return new FollowPathCommand(robot.drive, path_4Path);
    }

    /** Caps path speed (in/s) on one line of a path; from maxSpeed in the .turt file. */
    private com.pedropathing.config.Modifier maxVelocity(double inchesPerSec) {
        return ((com.pedropathing.algorithm.Foresight) robot.drive.algorithm())
                .config.maxVelocityConstraint.at(inchesPerSec);
    }
}
