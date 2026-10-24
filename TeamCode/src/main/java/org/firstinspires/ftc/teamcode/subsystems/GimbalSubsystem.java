package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.function.DoubleSupplier;

/**
 * Two-axis gimbal driven by the "pitch" and "roll" servos.
 *
 * Positions are given as -1..+1 (0 = centered) and mapped onto each servo's range
 * around its center position.
 */
public class GimbalSubsystem extends SubsystemBase {
    // Full rotation of the servo over positions 0..1.  TODO: check the servo spec (often 180, 270 or 300).
    private static final double SERVO_FULL_DEGREES = 180;

    // Maximum gimbal angle either side of center, in degrees (reached at full stick).
    private static final double PITCH_MAX_DEGREES = 15;
    private static final double ROLL_MAX_DEGREES  = 15;

    // Servo positions at center and the travel either side of center (0..1 servo units).
    private static final double PITCH_CENTER = 0.5;
    private static final double PITCH_RANGE  = PITCH_MAX_DEGREES / SERVO_FULL_DEGREES;
    private static final double ROLL_CENTER  = 0.5;
    private static final double ROLL_RANGE   = ROLL_MAX_DEGREES / SERVO_FULL_DEGREES;

    // +1 or -1 to set which way each servo turns for a positive angle.
    private static final double PITCH_DIRECTION = 1;
    private static final double ROLL_DIRECTION  = -1;

    private final OpMode opMode;
    private Servo pitchServo;
    private Servo rollServo;
    private double pitch;
    private double roll;

    // == standard Methods ===================================
    public GimbalSubsystem(OpMode myOpMode) {
        opMode = myOpMode;
    }

    public void init() {
        pitchServo = opMode.hardwareMap.get(Servo.class, "pitch");
        rollServo  = opMode.hardwareMap.get(Servo.class, "roll");
        center();
    }

    @Override
    public void periodic(){
        showTelemetry();
    }

    void showTelemetry() {
        opMode.telemetry.addData("GIMBAL", "pitch = %5.1f deg  roll = %5.1f deg",
                pitch * PITCH_MAX_DEGREES, roll * ROLL_MAX_DEGREES);
    }

    // == new elements =========================================

    /** Sets the gimbal angles, each -1..+1 (0 = centered). */
    public void setPosition(double newPitch, double newRoll) {
        pitch = Range.clip(newPitch, -1, 1);
        roll  = Range.clip(newRoll,  -1, 1);
        pitchServo.setPosition(PITCH_CENTER + PITCH_DIRECTION * pitch * PITCH_RANGE);
        rollServo.setPosition(ROLL_CENTER + ROLL_DIRECTION * roll * ROLL_RANGE);
    }

    public void center() {
        setPosition(0, 0);
    }

    // == Command methods  ==========================================

    /** Continuously follows the given inputs (e.g. a joystick) while scheduled. */
    public Command followCommand(DoubleSupplier pitchInput, DoubleSupplier rollInput) {
        return new RunCommand(() -> setPosition(pitchInput.getAsDouble(), rollInput.getAsDouble()), this);
    }

    public Command centerCommand() {
        return new InstantCommand(this::center, this);
    }
}
