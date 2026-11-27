package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.utils.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;

/**
 * Drives the robot straight back for pruneTimeMs, then straight forward for pruneTimeMs - 50 (so it
 * stays clear or any obstacles), then stops the drive.  Robot-centric manual drive, no strafe.
 * Complements JiggleCommand's side-to-side shake: used in auto to pull any pollen the flower
 * has released into the intake.  Total time is 2 x pruneTimeMs - 50.
 *
 * Both parameters are magnitudes (the sign of prunePower is ignored); the command picks the
 * directions itself: new PruneCommand(robot, 0.4, 250).
 *
 * The heading isn't held while it drives (manual drive, not Pedro), so the robot can turn a
 * little; the aimAndShootCommand() that follows in auto re-aims anyway.
 */
public class PruneCommand extends CommandBase {

    private final BiobuzzRobot robot;
    private final int pruneTimeMs;       // each direction
    private final double prunePower;     // magnitude
    private final Timer pruneTimer = new Timer();   // whole prune
    private boolean goingForward;

    public PruneCommand(BiobuzzRobot robot, double prunePower, int pruneTimeMs) {
        this.robot = robot;
        this.pruneTimeMs = Math.abs(pruneTimeMs);
        this.prunePower = Math.abs(prunePower);
    }

    @Override
    public void initialize() {
        pruneTimer.reset();
        goingForward = false;
        robot.drive.manual(-prunePower, 0, 0);
    }

    @Override
    public void execute() {
        if (!goingForward && pruneTimer.milliseconds() >= pruneTimeMs) {
            goingForward = true;
            robot.drive.manual(prunePower, 0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return pruneTimer.milliseconds() >= ((2 * pruneTimeMs) - 50);
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stop();
    }
}
