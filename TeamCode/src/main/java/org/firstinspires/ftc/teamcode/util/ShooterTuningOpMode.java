package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Shooter tuning: drive to a spot, adjust RPM / hood / turret by hand until shots land,
 * then copy the displayed distance, RPM and hood angle (degrees) into ShooterSubsystem.SHOT_TABLE.
 *
 * Start with the robot at TUNING_START_POSE and the turret centered.
 *
 *   Left / right sticks : drive (field centric, half speed)
 *   Dpad up / down      : flywheel RPM +/- 100
 *   Dpad left / right   : turret +/- 5 degrees
 *   Y / A               : hood +/- 1 degree
 *   B                   : toggle odometry tracking (uses the table) / manual
 *   X                   : toggle alliance
 *   Touchpad            : reset pose to TUNING_START_POSE
 */
@TeleOp(name = "Shooter Tuning", group = "Tuning")
public class ShooterTuningOpMode extends CommandOpMode {

    // TODO placeholder: a known, easy-to-reproduce starting spot on the field.
    private static final Pose TUNING_START_POSE = new Pose(72, 72, Math.toRadians(0));
    private BiobuzzRobot robot;
    private boolean      redAlliance = true;

    private double rpm       = 3000;
    private double hoodDeg   = 12;
    private double turretRad = 0;

    @Override
    public void initialize() {
        robot = new BiobuzzRobot(this);
        robot.initialize();
        robot.drive.setPose(TUNING_START_POSE);
        robot.shooter.setAlliance(redAlliance);

        GamepadEx driverOp = new GamepadEx(gamepad1);
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(new InstantCommand(() -> adjust(100, 0, 0)));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(new InstantCommand(() -> adjust(-100, 0, 0)));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new InstantCommand(() -> adjust(0, 0, Math.toRadians(5))));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(new InstantCommand(() -> adjust(0, 0, Math.toRadians(-5))));
        driverOp.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(new InstantCommand(() -> adjust(0, 1, 0)));
        driverOp.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(() -> adjust(0, -1, 0)));
        driverOp.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(new InstantCommand(this::toggleTracking));
        driverOp.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(new InstantCommand(() -> robot.shooter.setAlliance(redAlliance = !redAlliance)));
        driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD)
            .whenPressed(new InstantCommand(() -> robot.drive.setPose(TUNING_START_POSE)));
        // The intake collects by itself.
    }

    @Override
    public void run() {
        super.run();

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_x * 0.5,
                -gamepad1.right_stick_x * 0.5,
                robot.drive.pose().heading() - TUNING_START_POSE.heading());
        robot.drive.manual(powers);
        robot.drive.update();

        telemetry.addData("Pose", "%5.1f, %5.1f, %5.1f", robot.drive.pose().x(),
            robot.drive.pose().y(), Math.toDegrees(robot.drive.pose().heading()));
        telemetry.addData("Manual RPM : hood (deg) : turret", "%5.0f : %5.1f : %5.1f",
                rpm, hoodDeg, Math.toDegrees(turretRad));
        telemetry.addData("Hood now (deg)", "%5.1f", robot.shooter.getTargetHoodDeg());
        telemetry.update();
    }

    private void adjust(double dRpm, double dHoodDeg, double dTurret) {
        rpm       = Math.max(0, rpm + dRpm);
        hoodDeg   = Range.clip(hoodDeg + dHoodDeg, ShooterSubsystem.HOOD_MIN_DEG, ShooterSubsystem.HOOD_MAX_DEG);
        turretRad = turretRad + dTurret;
        if (robot.shooter.getMode() != ShooterSubsystem.Mode.TRACKING) {
            robot.shooter.setManual(rpm, hoodDeg, turretRad);
        }
    }

    private void toggleTracking() {
        if (robot.shooter.getMode() == ShooterSubsystem.Mode.TRACKING) {
            robot.shooter.setManual(rpm, hoodDeg, turretRad);
        } else {
            robot.shooter.track();
        }
    }
}
