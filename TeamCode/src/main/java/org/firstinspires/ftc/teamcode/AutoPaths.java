package org.firstinspires.ftc.teamcode;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

public class AutoPaths {
    private final Follower follower;
    private PoseFactory poseFactory;

    public AutoPaths(Follower sharedFollower) {
        follower = sharedFollower;
        poseFactory = PoseFactory.degrees();
    }

    // =================================================
    // Note:  Do not change anything above this line.
    //        Create a public method below for each file in the pp folder.
    // =================================================

    // Generated from FirstPath.turt
    public FollowPathCommand firstPath() {
        final Pose start = poseFactory.of(55.1229, 6.3902, 90);
        final Pose driveToShoot = poseFactory.of(55.1229, 29.3313, 180);

        Path firstPathPath = Paths.path(
                line(start, driveToShoot).linear(start, driveToShoot)
        );

        return new FollowPathCommand(follower, firstPathPath);
    }

    /** Where firstPath() starts.  Set the follower's pose to this once, before the auto runs. */
    public Pose firstPathStart() {
        return poseFactory.of(55.1229, 6.3902, 90);
    }

    // Generated from FourthPath.turt
    public FollowPathCommand fourthPath() {
        final Pose start = poseFactory.of(48.5918, 132.5373, 90);
        final Pose auto1StepD = poseFactory.of(57.9558, 112.9203, 90);

        Path fourthPathPath = Paths.path(
                line(start, auto1StepD).linear(start, auto1StepD)
        );

        return new FollowPathCommand(follower, fourthPathPath);
    }

    /** Where fourthPath() starts.  Set the follower's pose to this once, before the auto runs. */
    public Pose fourthPathStart() {
        return poseFactory.of(48.5918, 132.5373, 90);
    }

    // Generated from SecondPath.turt
    public FollowPathCommand secondPath() {
        final Pose start = poseFactory.of(56, 31.127, 90);
        final Pose auto1StepB = poseFactory.of(28.6009, 8, 180);
        final Pose auto1StepBControl1 = poseFactory.of(46.8904, 10.1514, 0);
        final Pose point2 = poseFactory.of(8, 8, 180);

        Path secondPathPath = Paths.path(
                curve(start, auto1StepBControl1, auto1StepB).linear(start, auto1StepB),
                line(auto1StepB, point2).linear(auto1StepB, point2)
        );

        return new FollowPathCommand(follower, secondPathPath);
    }

    /** Where secondPath() starts.  Set the follower's pose to this once, before the auto runs. */
    public Pose secondPathStart() {
        return poseFactory.of(56, 31.127, 90);
    }

    // Generated from ThirdPath.turt
    public FollowPathCommand thirdPath() {
        final Pose start = poseFactory.of(9.7456, 6.9364, 90);
        final Pose auto1StepC = poseFactory.of(33.2155, 97.4696, 90);
        final Pose auto1StepCControl1 = poseFactory.of(32.1014, 27.9417, 0);
        final Pose auto1StepCControl2 = poseFactory.of(22.4803, 74.705, 0);
        final Pose point2 = poseFactory.of(48.0318, 130.8613, 90);

        Path thirdPathPath = Paths.path(
                curve(start, auto1StepCControl1, auto1StepCControl2, auto1StepC).linear(Math.toRadians(180), Math.toRadians(90)),
                line(auto1StepC, point2).linear(Math.toRadians(180), Math.toRadians(90))
        );

        return new FollowPathCommand(follower, thirdPathPath);
    }

    /** Where thirdPath() starts.  Set the follower's pose to this once, before the auto runs. */
    public Pose thirdPathStart() {
        return poseFactory.of(9.7456, 6.9364, 90);
    }
}
