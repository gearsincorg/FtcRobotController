package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.utils.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;

/**
 * Drives the robot straight forward or backward (robot-centric manual drive, no strafe) at
 * prunePower (positive = forward, negative = back) for pruneTimeMs, then stops the drive.
 * Complements JiggleCommand's side-to-side shake: used in auto to pull any pollen the flower
 * has released into the intake.  Auto chains two, back then forward
 * (new PruneCommand(robot, -0.4, 250), new PruneCommand(robot, 0.4, 250)) before the volley.
 *
 * The heading isn't held while it drives (manual drive, not Pedro), so the robot can turn a
 * little; the aimAndShootCommand() that follows in auto re-aims anyway.
 */
public class PruneCommand extends CommandBase {

    private final BiobuzzRobot robot;
    private final int pruneTimeMs;
    private final double prunePower;
    private final Timer pruneTimer = new Timer();   // whole prune

    public PruneCommand(BiobuzzRobot robot, double prunePower, int pruneTimeMs) {
        this.robot = robot;
        this.pruneTimeMs = pruneTimeMs;
        this.prunePower = prunePower;
    }

    @Override
    public void initialize() {
        pruneTimer.reset();
        robot.drive.manual(prunePower, 0, 0);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return pruneTimer.milliseconds() >= pruneTimeMs;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stop();
    }
}
