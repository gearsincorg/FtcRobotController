package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

/**
 * Commands that need more than one subsystem, shared by teleop and auto.
 */
public final class RobotCommands {

    private RobotCommands() {}

    /**
     * Spin up and aim as soon as we're full; spin down once everything has been shot (or ejected).
     * Call once from initialize().  If only part of a load is fired, the shooter keeps tracking.
     */
    public static void bindAutoAim(ShooterSubsystem shooter, IntakeSubsystem intake) {
        new Trigger(intake::isFull)
            .whenActive(shooter.startTrackingCommand());
        new Trigger(() -> intake.isEmpty() && !intake.isShooting())
            .whenActive(shooter.idleCommand());
    }

    /** Aim, spin up and fire every ball; finishes when the last one has left. */
    public static Command shootAll(ShooterSubsystem shooter, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
            shooter.startTrackingCommand(),
            intake.shootAllCommand()
        );
    }

    /** Aim, spin up and fire one ball. */
    public static Command shootOne(ShooterSubsystem shooter, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
            shooter.startTrackingCommand(),
            intake.shootOneCommand()
        );
    }

    /** Collect while following a path; stops collecting when the path ends (or when full). */
    public static Command collectWhile(Command path, IntakeSubsystem intake) {
        return new ParallelDeadlineGroup(path, intake.intakeCommand());
    }
}
