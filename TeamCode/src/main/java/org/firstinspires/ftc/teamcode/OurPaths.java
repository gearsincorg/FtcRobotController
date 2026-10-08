package org.firstinspires.ftc.teamcode;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

public class OurPaths {
    private final Follower follower;
    private PoseFactory poseFactory;

    public OurPaths(Follower sharedFollower) {
        follower    = sharedFollower;
        poseFactory = PoseFactory.degrees();
    }

    public FollowPathCommand pathToFlower1() {
        final Pose start = poseFactory.of(56, 8, 90);
        final Pose path1 = poseFactory.of(38.1349, 31.1277, 180);
        final Pose path1Control1 = poseFactory.of(52.1743, 19.3243, 0);
        final Pose point2 = poseFactory.of(17.9835, 46.4498, 180);
        final Pose point3 = poseFactory.of(12.9928, 46.731, 180);

        follower.setPose(start);
        Path flowerPath = Paths.path(
            curve(start, path1Control1, path1).linear(start, path1),
            line(path1, point2).linear(path1, point2),
            line(point2, point3).linear(point2, point3)
        );

        return new FollowPathCommand(follower, flowerPath);
    }

    public FollowPathCommand pathToShoot(){
        final Pose point3 = poseFactory.of(12.9928, 46.731, 180);
        final Pose point4 = poseFactory.of(57.2496, 22.9785, 90);
        final Pose point4Control1 = poseFactory.of(27.8336, 15.0438, 0);

        Path flowerPath2 = Paths.path(
            curve(point3, point4Control1, point4).linear(point3,point4)
        );

        return new FollowPathCommand(follower, flowerPath2);
    }
}
