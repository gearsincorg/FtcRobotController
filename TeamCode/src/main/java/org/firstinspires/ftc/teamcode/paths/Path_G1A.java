package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;
import org.firstinspires.ftc.teamcode.Commands.JiggleCommand;
import org.firstinspires.ftc.teamcode.Commands.PruneCommand;

public class Path_G1A {
    private PoseFactory poseFactory;
    private BiobuzzRobot robot;

    public Path_G1A(BiobuzzRobot robot) {
        this.robot = robot;
        poseFactory = PoseFactory.degrees();
    }

    public Command autoCommand() {

        SequentialCommandGroup auto = new SequentialCommandGroup();
        robot.setStartPose(getStartPose());

        // Add paths and other subsystem commands here
        auto.addCommands(robot.shooter.startTrackingCommand(), robot.aimAndShootTransferOnlyCommand());  // shoot everything.
        auto.addCommands(path_start(), robot.intake.intakeUntilFullCommand(1000));
        auto.addCommands(path_1()); // go and check on hive tilt.
        auto.addCommands(new ConditionalCommand(
            //  Hive is still up, so go get more pollen
            new SequentialCommandGroup(
                robot.aimAndShootCommand(), robot.intake.flowerDownCommand(), path_5().withTimeout(1500),
                new ParallelDeadlineGroup(robot.intake.intakeUntilFullCommand(1000), new JiggleCommand(robot, 1000) ),
                new ParallelCommandGroup(path_6().withTimeout(5000), new SequentialCommandGroup(robot.intake.flowerBumpCommand(), new WaitCommand(1600),robot.intake.flowerDownCommand())),
                new ParallelCommandGroup(new JiggleCommand(robot, 1000), robot.aimAndShootCommand()),
                new PruneCommand(robot, 0.4, 250),robot.aimAndShootCommand(2000)
                                       ),
            //  Hive has tipped so go and score other side.
            new SequentialCommandGroup(
                new ParallelCommandGroup(path_2().withTimeout(6000), new SequentialCommandGroup(robot.intake.flowerBumpCommand(), new WaitCommand(1800),robot.intake.flowerDownCommand())),
                                       new ParallelCommandGroup(new JiggleCommand(robot, 1000), robot.aimAndShootCommand()),
                                       new PruneCommand(robot, 0.4, 250),robot.aimAndShootCommand(2000)
                                       ),
            robot.vision::isHiveUp
        ));

        auto.addCommands(robot.shooter.stopTrackingCommand());
        return auto;
    }

    // =================================================
    // Note:  Do not change anything above this line.
    //        Create a public method below for each file in the pp folder.
    // =================================================

    // Generated from G1A_start.turt
    public Pose getStartPose() {
        return poseFactory.of(30.88, 10, 270); // from start path in this name group
    }

    public FollowPathCommand path_start() {
        final Pose start = getStartPose();
        final Pose toGarden = poseFactory.of(8, 10, 270);
        final Pose toGardenControl1 = poseFactory.of(19.1833, 22.3761, 0);

        Path path_startPath = Paths.path(
                curve(start, toGardenControl1, toGarden).linear(start, toGarden).with(maxVelocity(30))
        );

        return new NamedFollowPathCommand(robot, "G1A_start", new String[] {"toGarden"}, path_startPath);
    }

    // Generated from G1A_1.turt
    public FollowPathCommand path_1() {
        final Pose start = poseFactory.of(8, 10, 270);
        final Pose tiltCheck = poseFactory.of(32, 24, 90);
        final Pose tiltCheckControl1 = poseFactory.of(19.1833, 22.3761, 0);

        Path path_1Path = Paths.path(
                curve(start, tiltCheckControl1, tiltCheck).linear(start, tiltCheck)
        );

        return new NamedFollowPathCommand(robot, "G1A_1", new String[] {"tiltCheck"}, path_1Path);
    }

    // Generated from G1A_2.turt
    public FollowPathCommand path_2() {
        final Pose start = poseFactory.of(32, 24, 90);
        final Pose toFlower = poseFactory.of(32, 110, 90);
        final Pose point2 = poseFactory.of(48, 130, 90);
        final Pose point2Control1 = poseFactory.of(32, 120.4434, 0);
        final Pose point2Control2 = poseFactory.of(47.7116, 118.5408, 0);

        Path path_2Path = Paths.path(
                line(start, toFlower).linear(start, toFlower).with(maxVelocity(55)),
                curve(toFlower, point2Control1, point2Control2, point2).linear(toFlower, point2).with(maxVelocity(40))
        );

        return new NamedFollowPathCommand(robot, "G1A_2", new String[] {"toFlower", "point2"}, path_2Path);
    }

    // Generated from G1A_5.turt
    public FollowPathCommand path_5() {
        final Pose start = poseFactory.of(36, 24, 90);
        final Pose toFlower = poseFactory.of(15, 48, 180);
        final Pose toFlowerControl1 = poseFactory.of(21.7711, 41.5611, 0);
        final Pose toFlowerControl2 = poseFactory.of(19.5736, 49.5015, 0);

        Path path_5Path = Paths.path(
                curve(start, toFlowerControl1, toFlowerControl2, toFlower).linear(start, toFlower).with(maxVelocity(40))
        );

        return new NamedFollowPathCommand(robot, "G1A_5", new String[] {"toFlower"}, path_5Path);
    }

    // Generated from G1A_6.turt
    public FollowPathCommand path_6() {
        final Pose start = poseFactory.of(15, 48, 180);
        final Pose toFlower = poseFactory.of(32, 56, 90);
        final Pose toFlowerControl1 = poseFactory.of(27.9812, 48, 0);
        final Pose toFlowerControl2 = poseFactory.of(31.0172, 52, 0);
        final Pose point2 = poseFactory.of(32, 103.6793, 90);
        final Pose point3 = poseFactory.of(48, 128, 90);
        final Pose point3Control1 = poseFactory.of(36, 123.6474, 0);
        final Pose point3Control2 = poseFactory.of(49, 112.1035, 0);

        Path path_6Path = Paths.path(
                curve(start, toFlowerControl1, toFlowerControl2, toFlower).linear(start, toFlower).with(maxVelocity(30)),
                line(toFlower, point2).linear(toFlower, point2).with(maxVelocity(55)),
                curve(point2, point3Control1, point3Control2, point3).linear(point2, point3).with(maxVelocity(40))
        );

        return new NamedFollowPathCommand(robot, "G1A_6", new String[] {"toFlower", "point2", "point3"}, path_6Path);
    }

    /** Caps path speed (in/s) on one line of a path; from maxSpeed in the .turt file. */
    private com.pedropathing.config.Modifier maxVelocity(double inchesPerSec) {
        return ((com.pedropathing.algorithm.Foresight) robot.drive.algorithm())
                .config.maxVelocityConstraint.at(inchesPerSec);
    }
}
