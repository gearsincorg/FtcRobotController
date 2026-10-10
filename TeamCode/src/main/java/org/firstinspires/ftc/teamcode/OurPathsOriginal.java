package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

public class OurPathsOriginal {
    private final Follower follower;
    private PoseFactory poseFactory;

    public OurPathsOriginal(Follower sharedFollower) {
        follower = sharedFollower;
        poseFactory = PoseFactory.degrees();
    }

    // =================================================
    // Note:  Do not change anything above this line.
    //        Create a public method below for each file in the pp folder.
    // =================================================

    // Generated from Auto1.pp
    public FollowPathCommand auto1() {
        final Pose start = poseFactory.of(56, 8, 90);
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

        return new FollowPathCommand(follower, auto1Path);
    }
}
