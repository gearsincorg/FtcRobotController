package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot;

public class Path_FarCenter {
    private PoseFactory poseFactory;
    private Robot robot;

    public Path_FarCenter(Robot robot) {
        this.robot = robot;
        poseFactory = PoseFactory.degrees();
    }

    public Command autoCommand() {

        SequentialCommandGroup auto = new SequentialCommandGroup();
        robot.drive.setPose(getStartPose());

        // Add paths and other subsystem commands here
         auto.addCommands(path_start());
        return auto;
    }

    // =================================================
    // Note:  Do not change anything above this line.
    //        Create a public method below for each file in the pp folder.
    // =================================================

    // Generated from FarCenter_start.turt
    public Pose getStartPose() {
        return poseFactory.of(65.6446, 134.7929, 90); // from start path in this name group
    }

    public FollowPathCommand path_start() {
        final Pose start = getStartPose();
        final Pose point1 = poseFactory.of(48.6468, 130.9488, 90);
        final Pose point1Control1 = poseFactory.of(65.6446, 123.1737, 0);
        final Pose point1Control2 = poseFactory.of(47.0432, 127.4543, 0);

        Path path_startPath = Paths.path(
                curve(start, point1Control1, point1Control2, point1).linear(Math.toRadians(90), Math.toRadians(90))
        );

        return new FollowPathCommand(robot.drive, path_startPath);
    }
}
