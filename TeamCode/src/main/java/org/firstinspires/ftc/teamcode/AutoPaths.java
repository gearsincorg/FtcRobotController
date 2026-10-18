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

    // Generated from Auto1.pp
    public FollowPathCommand auto1() {
        final Pose start = poseFactory.of(56, 9, 90);
        final Pose path1 = poseFactory.of(57.9551, 27.2021, 90);
        final Pose point2 = poseFactory.of(58.5665, 75.3092, 89.2719);
        final Pose point3 = poseFactory.of(118.5933, 94.3212, -3.2857);
        final Pose point3Control1 = poseFactory.of(58.6762, 119.1865, 0);
        final Pose point3Control2 = poseFactory.of(104.1416, 94.9352, 0);
        final Pose point4 = poseFactory.of(128.7824, 94.0026, -1.7913);
        final Pose point5 = poseFactory.of(119.0993, 94.1641, -0.9554);

        Path auto1Path = Paths.path(
                line(start, path1).linear(start, path1),
                line(path1, point2).tangent(),
                curve(point2, point3Control1, point3Control2, point3).tangent(),
                line(point3, point4).tangent(),
                line(point4, point5).reverseTangent()
        );

        follower.setPose(start);
        return new FollowPathCommand(follower, auto1Path);
    }

    // Generated from Auto2.pp
    public FollowPathCommand auto2() {
        final Pose start = poseFactory.of(56, 8, 90);
        final Pose path1 = poseFactory.of(56, 36, 180);
        final Pose point2 = poseFactory.of(21.4646, 68.0976, 180);
        final Pose point2Control1 = poseFactory.of(56.095, 75.0648, 0);

        Path auto2Path = Paths.path(
                line(start, path1).linear(start, path1),
                curve(path1, point2Control1, point2).constant(point2)
        );

        follower.setPose(start);
        return new FollowPathCommand(follower, auto2Path);
    }

    // Generated from FirstPath.pp
    public FollowPathCommand firstPath() {
        final Pose start = poseFactory.of(56, 8, 90);
        final Pose auto1StepA = poseFactory.of(56, 31.1277, 90);

        Path firstPathPath = Paths.path(
                line(start, auto1StepA).linear(start, auto1StepA)
        );

        follower.setPose(start);
        return new FollowPathCommand(follower, firstPathPath);
    }

    // Generated from FourthPath.pp
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

    // Generated from SecondPath.pp
    public FollowPathCommand secondPath() {
        final Pose start = poseFactory.of(56, 31.127, 90);
        final Pose auto1StepB = poseFactory.of(14.7884, 46.1506, 180);

        Path secondPathPath = Paths.path(
                line(start, auto1StepB).linear(start, auto1StepB)
        );

        follower.setPose(start);
        return new FollowPathCommand(follower, secondPathPath);
    }

    // Generated from ThirdPath.pp
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
