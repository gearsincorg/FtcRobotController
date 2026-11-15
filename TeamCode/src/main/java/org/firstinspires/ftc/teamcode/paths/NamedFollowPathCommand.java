package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;

/**
 * A FollowPathCommand that also tells the robot which path it is (the .turt file name) and the
 * names of its lines (each line's end point in Turtle Tracer), so telemetry can show where auto is.
 * tools/turt2java.py generates these; it behaves exactly like FollowPathCommand otherwise.
 */
public class NamedFollowPathCommand extends FollowPathCommand {
    private final BiobuzzRobot robot;
    private final String name;
    private final String[] lineNames;

    public NamedFollowPathCommand(BiobuzzRobot robot, String name, String[] lineNames, Path path) {
        super(robot.drive, path);
        this.robot = robot;
        this.name = name;
        this.lineNames = lineNames;
    }

    @Override
    public void initialize() {
        super.initialize();
        robot.setCurrentPath(name, lineNames);
    }
}
