package org.firstinspires.ftc.teamcode.paths;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.AutoConfig;

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

        return auto;
    }


    // =================================================
    // Note:  Do not change anything above this line.
    //        Create a public method below for each file in the pp folder.
    // =================================================

    // Generated from NearCenter_1.turt
    public Pose getStartPose() {
        return poseFactory.of(55.1229, 6.3902, 90); // from start path in this name group
    }

    public FollowPathCommand path_start() {
        final Pose start = getStartPose();
        final Pose driveToShoot = poseFactory.of(55.1229, 29.3313, 180);

        Path nearCenter1Path = Paths.path(
                line(start, driveToShoot).linear(start, driveToShoot)
        );

        return new FollowPathCommand(robot.drive, nearCenter1Path);
    }

    // Generated from NearCenter_2.turt
    public FollowPathCommand nearCenter2() {
        final Pose start = poseFactory.of(56, 31.127, 90);
        final Pose auto1StepB = poseFactory.of(28.6009, 8, 180);
        final Pose auto1StepBControl1 = poseFactory.of(46.8904, 10.1514, 0);
        final Pose point2 = poseFactory.of(8, 8, 180);

        Path nearCenter2Path = Paths.path(
                curve(start, auto1StepBControl1, auto1StepB).linear(start, auto1StepB),
                line(auto1StepB, point2).linear(auto1StepB, point2)
        );

        return new FollowPathCommand(robot.drive, nearCenter2Path);
    }

    /** Where nearCenter2() starts.  Set the follower's pose to this once, before the auto runs. */
    public Pose nearCenter2Start() {
        return poseFactory.of(56, 31.127, 90);
    }

    // Generated from NearCenter_3.turt
    public FollowPathCommand nearCenter3() {
        final Pose start = poseFactory.of(9.7456, 6.9364, 90);
        final Pose auto1StepC = poseFactory.of(33.2155, 97.4696, 90);
        final Pose auto1StepCControl1 = poseFactory.of(32.1014, 27.9417, 0);
        final Pose auto1StepCControl2 = poseFactory.of(22.4803, 74.705, 0);
        final Pose point2 = poseFactory.of(48.0318, 130.8613, 90);

        Path nearCenter3Path = Paths.path(
                curve(start, auto1StepCControl1, auto1StepCControl2, auto1StepC).linear(Math.toRadians(180), Math.toRadians(90)),
                line(auto1StepC, point2).linear(Math.toRadians(180), Math.toRadians(90))
        );

        return new FollowPathCommand(robot.drive, nearCenter3Path);
    }

    // Generated from NearCenter_4.turt
    public FollowPathCommand nearCenter4() {
        final Pose start = poseFactory.of(48.5918, 132.5373, 90);
        final Pose auto1StepD = poseFactory.of(57.9558, 112.9203, 90);

        Path nearCenter4Path = Paths.path(
                line(start, auto1StepD).linear(start, auto1StepD)
        );

        return new FollowPathCommand(robot.drive, nearCenter4Path);
    }
}
