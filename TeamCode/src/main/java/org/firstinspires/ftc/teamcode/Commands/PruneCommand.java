package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.utils.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;

/**
 * Strafes the robot left and right (robot-centric, manual drive) to shake loose balls into the
 * intake.  It starts with half a stroke left, then full strokes right / left, so it stays centred
 * on where it started.  Ends after jiggleTimeMs and stops the drive; in auto it runs in a
 * ParallelDeadlineGroup with intake.intakeUntilFullCommand(), so it stops as soon as the intake
 * is full.
 *
 * The heading isn't held while it strafes (manual drive, not Pedro), so the robot can turn a
 * little; the aimAndShootCommand() that follows in auto re-aims anyway.
 */
public class PruneCommand extends CommandBase {

    private final BiobuzzRobot robot;
    private final int pruneTimeMs;
    private final double prunePower;
    private final Timer pruneTimer = new Timer();   // whole jiggle

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
