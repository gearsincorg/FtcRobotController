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
public class JiggleCommand extends CommandBase {
    private static final double JIGGLE_POWER = 0.3;   // strafe power
    private static final int    STROKE_MS    = 200;   // one full stroke (left or right)

    private final BiobuzzRobot robot;
    private final int          jiggleTimeMs;
    private final Timer        jiggleTimer = new Timer();   // whole jiggle
    private final Timer        strokeTimer = new Timer();   // current stroke
    private boolean movingLeft;
    private boolean firstStroke;

    public JiggleCommand(BiobuzzRobot robot, int jiggleTimeMs) {
        this.robot = robot;
        this.jiggleTimeMs = jiggleTimeMs;
    }

    @Override
    public void initialize() {
        movingLeft = true;
        firstStroke = true;
        jiggleTimer.reset();
        strokeTimer.reset();
        robot.drive.manual(0, JIGGLE_POWER, 0);
    }

    @Override
    public void execute() {
        // The first stroke is half length so the jiggle is centred on the start.
        if (firstStroke && strokeTimer.milliseconds() >= STROKE_MS / 2.0) {
            robot.drive.manual(0, -JIGGLE_POWER, 0);
            movingLeft = false;
            firstStroke = false;
            strokeTimer.reset();
        } else if (strokeTimer.milliseconds() >= STROKE_MS) {
            movingLeft = !movingLeft;
            robot.drive.manual(0, movingLeft ? JIGGLE_POWER : -JIGGLE_POWER, 0);
            strokeTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return jiggleTimer.milliseconds() >= jiggleTimeMs;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stop();
    }
}
