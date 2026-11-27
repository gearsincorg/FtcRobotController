package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;

/**
 * Commands that need more than one subsystem, shared by teleop and auto.
 */
public final class RobotCommands {

    private RobotCommands() {}

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
}

