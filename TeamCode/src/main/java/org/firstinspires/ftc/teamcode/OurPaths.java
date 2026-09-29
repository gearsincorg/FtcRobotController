package org.firstinspires.ftc.teamcode;
import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class OurPaths {
    final Follower follower;

    public OurPaths(Follower follower){
        this.follower = follower;
    }

    public Command PathToFlower() {
        final PoseFactory poseFactory = PoseFactory.degrees();

        final Pose start = poseFactory.of(56, 8, 90);
        final Pose path1 = poseFactory.of(34.2776, 25.0373, 180);
        final Pose path1Control1 = poseFactory.of(41.7192, 4.9104, 0);
        final Pose point2 = poseFactory.of(17.9835, 46.4498, 180);

        follower.setPose(start);
        Path flowerPath = Paths.path(
            curve(start, path1Control1, path1).linear(start, path1),
            line(path1, point2).linear(path1, point2)
        );
        return follow(follower, flowerPath);
    }
}
