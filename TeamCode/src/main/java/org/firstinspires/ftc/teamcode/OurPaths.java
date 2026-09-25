package org.firstinspires.ftc.teamcode;
import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedro.Constants;


public class OurPaths {
    Follower follower;

    public OurPaths(Follower follower){
        this.follower = follower;
    }

    public class Path1 {

         private final PoseFactory poseFactory = PoseFactory.degrees();
         private final Pose start = poseFactory.of(56, 8, 90);
         private final Pose path1 = poseFactory.of(34.2776, 25.0373, 180);
         private final Pose path1Control1 = poseFactory.of(39.08, 8.1586, 0);

        public Path1(){
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
        }


        public Path path1() {
            return curve(start, path1Control1, path1).linear(start, path1);
        }

        public Command autoRoutine() {
            return follow(follower, path1());
        }
    }


    public class PathToFlower {

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(56, 8, 90);
        private final Pose path1 = poseFactory.of(34.2776, 25.0373, 180);
        private final Pose path1Control1 = poseFactory.of(41.7192, 4.9104, 0);
        private final Pose point2 = poseFactory.of(17.9835, 46.4498, 180);

        public PathToFlower(){
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
        }

        public Path path1() {
            return curve(start, path1Control1, path1).linear(start, path1);
        }

        public Path path2() {
            return line(path1, point2).linear(path1, point2);
        }

        public Command autoRoutine() {
            return sequential(
                follow(follower, path1()),
                follow(follower, path2())
            );
        }
    }
}
