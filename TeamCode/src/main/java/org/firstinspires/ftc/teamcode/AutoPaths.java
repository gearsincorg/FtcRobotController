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
        final Pose start = poseFactory.of(56, 8, 90);
        final Pose auto1StepA = poseFactory.of(56, 31.1277, 90);

        Path firstPathPath = Paths.path(
                line(start, auto1StepA).linear(start, auto1StepA)
        );

        follower.setPose(start);
        return new FollowPathCommand(follower, firstPathPath);
    }

    // Generated from FourthPath.turt
    public FollowPathCommand fourthPath() {
        final Pose start = poseFactory.of(58.2418, 23.4656, 90);
        final Pose auto1StepD = poseFactory.of(27.1722, 49.3989, 90);
        final Pose auto1StepE = poseFactory.of(47.4362, 128.4376, 90);
        final Pose auto1StepEControl1 = poseFactory.of(26.7202, 98.6621, 0);

        Path fourthPathPath = Paths.path(
                line(start, auto1StepD).linear(start, auto1StepD),
                curve(auto1StepD, auto1StepEControl1, auto1StepE).linear(auto1StepD, auto1StepE)
        );

        follower.setPose(start);
        return new FollowPathCommand(follower, fourthPathPath);
    }

    // Generated from SecondPath.turt
    public FollowPathCommand secondPath() {
        final Pose start = poseFactory.of(56, 31.127, 90);
        final Pose auto1StepB = poseFactory.of(14.7884, 46.1506, 180);

        Path secondPathPath = Paths.path(
                line(start, auto1StepB).linear(start, auto1StepB)
        );

        follower.setPose(start);
        return new FollowPathCommand(follower, secondPathPath);
    }

    // Generated from ThirdPath.turt
    public FollowPathCommand thirdPath() {
        final Pose start = poseFactory.of(15, 46, 90);
        final Pose auto1StepC = poseFactory.of(58.0301, 23.2102, 90);

        Path thirdPathPath = Paths.path(
                line(start, auto1StepC).linear(Math.toRadians(180), Math.toRadians(90))
        );

        follower.setPose(start);
        return new FollowPathCommand(follower, thirdPathPath);
    }
}
