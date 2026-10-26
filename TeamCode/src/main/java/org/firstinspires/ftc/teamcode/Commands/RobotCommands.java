package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Commands that need more than one subsystem, shared by teleop and auto.
 */
public final class RobotCommands {

    private RobotCommands() {}

    /**
     * Spin up and aim as soon as we're full; spin down once everything has been shot (or ejected).
     * Call once from initialize().  If only part of a load is fired, the shooter keeps tracking.
     */
    public static void bindAutoAim(BiobuzzRobot robot) {
        new Trigger(robot.intake::isFull)
            .whenActive(robot.shooter.startTrackingCommand());
        new Trigger(() -> robot.intake.isEmpty() && !robot.intake.isShooting())
            .whenActive(robot.shooter.idleCommand());
    }

    /** Aim, spin up and fire every ball in one volley; finishes when the last one has left. */
    public static Command shootAll(BiobuzzRobot robot) {
        return new SequentialCommandGroup(
            robot.shooter.startTrackingCommand(),
            robot.intake.shootAllCommand()
        );
    }

    /** Same, but gives up after timeoutMs as a safety net (for auto). */
    public static Command shootVolley(BiobuzzRobot robot, long timeoutMs) {
        return shootAll(robot).withTimeout(timeoutMs);
    }

    /** Aim, spin up and fire while scheduled (teleop: while the button is held). */
    public static Command shootWhileHeld(BiobuzzRobot robot) {
        return new SequentialCommandGroup(
            robot.shooter.startTrackingCommand(),
            robot.intake.shootWhileHeldCommand()
        );
    }

    /** Collect while following a path; stops collecting when the path ends (or when full). */
    public static Command collectWhile(Command path, IntakeSubsystem intake) {
        return new ParallelDeadlineGroup(path, intake.intakeCommand());
    }
}