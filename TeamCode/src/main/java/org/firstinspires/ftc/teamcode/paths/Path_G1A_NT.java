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
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;
import org.firstinspires.ftc.teamcode.Commands.JiggleCommand;

public class Path_G1A_NT {
    private PoseFactory poseFactory;
    private BiobuzzRobot robot;

    public Path_G1A_NT(BiobuzzRobot robot) {
        this.robot = robot;
        poseFactory = PoseFactory.degrees();
    }

    public Command autoCommand() {

        SequentialCommandGroup auto = new SequentialCommandGroup();
        robot.setStartPose(getStartPose());

        // Add paths and other subsystem commands here
        auto.addCommands(robot.shooter.startTrackingCommand(), path_start(), robot.aimAndShootCommand());
        auto.addCommands(path_1().withTimeout(2000), robot.intake.intakeUntilFullCommand(1000));
        auto.addCommands(path_2(), new ParallelCommandGroup( robot.intake.flowerDownCommand(), robot.aimAndShootCommand()));
        auto.addCommands(path_3().withTimeout(1500));
        auto.addCommands(new ParallelDeadlineGroup(robot.intake.intakeUntilFullCommand(2000), new JiggleCommand(robot, 2000)), robot.aimAndShootCommand());
        auto.addCommands(new ParallelCommandGroup( path_4().withTimeout(6000),
                            new SequentialCommandGroup(new WaitCommand(200), robot.intake.flowerUpCommand(),new WaitCommand(1500),robot.intake.flowerDownCommand())));
        auto.addCommands(new ParallelDeadlineGroup(robot.intake.intakeUntilFullCommand(1000), new JiggleCommand(robot, 1000)));
        auto.addCommands(path_5(),new ParallelCommandGroup(robot.aimAndShootCommand(), robot.intake.flowerUpCommand()));
        auto.addCommands(robot.shooter.stopTrackingCommand());
        return auto;
    }

    // =================================================
    // Note:  Do not change anything above this line.
    //        Create a public method below for each file in the pp folder.
    // =================================================

    // Generated from G1A_NT_start.turt
    public Pose getStartPose() {
        return poseFactory.of(30.875, 10, 270); // from start path in this name group
    }

    public FollowPathCommand path_start() {
        final Pose start = getStartPose();
        final Pose garden = poseFactory.of(30.875, 16, 230);

        Path path_startPath = Paths.path(
                line(start, garden).linear(start, garden).with(maxVelocity(30))
        );

        return new NamedFollowPathCommand(robot, "G1A_NT_start", new String[] {"garden"}, path_startPath);
    }

    // Generated from G1A_NT_1.turt
    public FollowPathCommand path_1() {
        final Pose start = poseFactory.of(30.875, 16, 230);
        final Pose garden = poseFactory.of(8, 10, 270);
        final Pose gardenControl1 = poseFactory.of(13.2224, 16, 0);

        Path path_1Path = Paths.path(
                curve(start, gardenControl1, garden).linear(start, garden).with(maxVelocity(30))
        );

        return new NamedFollowPathCommand(robot, "G1A_NT_1", new String[] {"garden"}, path_1Path);
    }

    // Generated from G1A_NT_2.turt
    public FollowPathCommand path_2() {
        final Pose start = poseFactory.of(8, 10, 270);
        final Pose transition = poseFactory.of(32.0671, 33.3781, 90);
        final Pose transitionControl1 = poseFactory.of(25, 17.8438, 0);
        final Pose transitionControl2 = poseFactory.of(32.0671, 15.8394, 0);
        final Pose point2 = poseFactory.of(32.0671, 106.0096, 90);
        final Pose point3 = poseFactory.of(44, 120, 110);
        final Pose point3Control1 = poseFactory.of(32.0671, 115.4811, 0);

        Path path_2Path = Paths.path(
                curve(start, transitionControl1, transitionControl2, transition).linear(start, transition).with(maxVelocity(30)),
                line(transition, point2).linear(transition, point2).with(maxVelocity(55)),
                curve(point2, point3Control1, point3).linear(point2, point3).with(maxVelocity(30))
        );

        return new NamedFollowPathCommand(robot, "G1A_NT_2", new String[] {"transition", "point2", "point3"}, path_2Path);
    }

    // Generated from G1A_NT_3.turt
    public FollowPathCommand path_3() {
        final Pose start = poseFactory.of(44, 120, 110);
        final Pose nearFlower = poseFactory.of(48, 125.2, 90);
        final Pose point2 = poseFactory.of(48, 128, 90);

        Path path_3Path = Paths.path(
                line(start, nearFlower).linear(start, nearFlower).with(maxVelocity(45)),
                line(nearFlower, point2).linear(nearFlower, point2)
        );

        return new NamedFollowPathCommand(robot, "G1A_NT_3", new String[] {"nearFlower", "point2"}, path_3Path);
    }

    // Generated from G1A_NT_4.turt
    public FollowPathCommand path_4() {
        final Pose start = poseFactory.of(48, 128, 90);
        final Pose nearShoot = poseFactory.of(15, 48, 180);
        final Pose nearShootControl1 = poseFactory.of(21.0298, 113.1373, 0);
        final Pose nearShootControl2 = poseFactory.of(46.2022, 46.4592, 0);

        Path path_4Path = Paths.path(
                curve(start, nearShootControl1, nearShootControl2, nearShoot).linear(start, nearShoot).with(maxVelocity(55))
        );

        return new NamedFollowPathCommand(robot, "G1A_NT_4", new String[] {"nearShoot"}, path_4Path);
    }

    // Generated from G1A_NT_5.turt
    public FollowPathCommand path_5() {
        final Pose start = poseFactory.of(15, 48, 180);
        final Pose nearShoot = poseFactory.of(25, 34, 210);

        Path path_5Path = Paths.path(
                line(start, nearShoot).linear(start, nearShoot).with(maxVelocity(45))
        );

        return new NamedFollowPathCommand(robot, "G1A_NT_5", new String[] {"nearShoot"}, path_5Path);
    }

    /** Caps path speed (in/s) on one line of a path; from maxSpeed in the .turt file. */
    private com.pedropathing.config.Modifier maxVelocity(double inchesPerSec) {
        return ((com.pedropathing.algorithm.Foresight) robot.drive.algorithm())
                .config.maxVelocityConstraint.at(inchesPerSec);
    }
}
