package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    // subsystem devices
    private DcMotorEx front;
    private DcMotorEx rear;
    private Servo hood;

    // Subsystem Constants
    private final double SHOOTER_COUNTS_TO_MPS = 0.072 * Math.PI / 28;
    private final double SERVO_GEAR_RATIO        =  24.0 / 90.0;
    public final double SHOOTER_ANGLE_MAX        =  37.0;
    public final double SHOOTER_ANGLE_MIN        = -37.0;
    private final double PULSE_SCALE_FACTOR      =  1.8e-3;   // make this match the spindexer in 2 places once servo is reprogrammed
    private final double SHOOTER_SPEED_TOLERANCE =  0.2;
    private final double MAX_MPS                 =  16.0;

    private final double P_STEP                  =  1.0;
    private final double F_STEP                  =  0.1;

    private final double SHOOTER_OFFSET          = -4.0;    // used to ensure that zero degrees is level.

    // General Subsystem Members
    private double  tiltAngle         = 0;
    private double  shooterServoValue = 0;
    private double  currentFrontMPS   = 0;
    private double  currentRearMPS    = 0;
    private double  targetFrontMPS    = 0;
    private double  targetRearMPS     = 0;
    private PIDFCoefficients shooterCoefs = new PIDFCoefficients(32,0,0,11.6);   //

    public boolean  atSpeed = false;

    public ShooterSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);  // do not remove

        front = myOpMode.hardwareMap.get(DcMotorEx.class, "frontWheel");
        front.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setPIDFCoefficients(RUN_USING_ENCODER, shooterCoefs);
        front.setMode(RUN_USING_ENCODER);
        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rear = myOpMode.hardwareMap.get(DcMotorEx.class, "rearWheel");
        rear.setDirection(DcMotorSimple.Direction.FORWARD);
        rear.setPIDFCoefficients(RUN_USING_ENCODER, shooterCoefs);
        rear.setMode(RUN_USING_ENCODER);
        rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood = myOpMode.hardwareMap.get(Servo.class, "hood");
    }

    @Override
    public void readSensors() {
        currentFrontMPS = front.getVelocity()   * SHOOTER_COUNTS_TO_MPS;
        currentRearMPS  = rear.getVelocity() * SHOOTER_COUNTS_TO_MPS;
    }

    @Override
    public void runProcessing() {

        /*
        //  This code lets us tune the PF coefficients
        boolean changedPIDF = false;

        // DETERMINE MANUAL shooter speed
        if (myOpMode.gamepad2.dpadUpWasPressed()) {
            shooterCoefs.f += F_STEP;
            changedPIDF = true;
        } else if (myOpMode.gamepad2.dpadDownWasPressed() && (shooterCoefs.f > 0)) {
            shooterCoefs.f -= F_STEP;
            changedPIDF = true;
        }

        // DETERMINE MANUAL shooter TILT
        if (myOpMode.gamepad2.dpadRightWasPressed()) {
            shooterCoefs.p += P_STEP;
            changedPIDF = true;
        } else if (myOpMode.gamepad2.dpadLeftWasPressed()  && (shooterCoefs.p > 0)) {
            shooterCoefs.p -= P_STEP;
            changedPIDF = true;
        }

        if (changedPIDF) {
            front.setPIDFCoefficients(RUN_USING_ENCODER, shooterCoefs);
            rear.setPIDFCoefficients(RUN_USING_ENCODER, shooterCoefs);
        }
        */

        shooterServoValue = MathUtils.clamp(0.5 - (tiltAngle * PULSE_SCALE_FACTOR / SERVO_GEAR_RATIO), 0.22, 0.78);  // make this match the spindexer in 2 places once servo is reprogrammed
        hood.setPosition(shooterServoValue);

        // slam on the breaks if we are just too fast
        if ((currentFrontMPS - targetFrontMPS) < 0.5) {
            front.setVelocity(targetFrontMPS / SHOOTER_COUNTS_TO_MPS);
        } else {
            front.setVelocity(0);
        }
        if ((currentRearMPS - targetRearMPS) < 0.5) {
            rear.setVelocity(targetRearMPS / SHOOTER_COUNTS_TO_MPS);
        } else {
            rear.setVelocity(0);
        }

        atSpeed = ((Math.abs(targetFrontMPS - currentFrontMPS) < SHOOTER_SPEED_TOLERANCE) &&
                (Math.abs(targetRearMPS - currentRearMPS) < SHOOTER_SPEED_TOLERANCE));
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("SHOOTER",   "F=%4.2f  B=%4.2f %s Tilt= %.0f", currentFrontMPS, currentRearMPS, atSpeed ? "OK" : "SLOW", tiltAngle);
        LoggingSubsystem.updateWheelSpeed(currentFrontMPS, currentRearMPS);

        // myOpMode.telemetry.addData("SET VEL",   "F=%4.2f  B=%4.2f", targetFrontMPS, targetRearMPS);
        // myOpMode.telemetry.addData("PIDF",   "P=%4.2f  I=%4.2f  D=%4.2f  F=%4.2f", shooterCoefs.p, shooterCoefs.i, shooterCoefs.d, shooterCoefs.f );
    }

    public void setAngle(double angle){
        tiltAngle = MathUtils.clamp(angle + SHOOTER_OFFSET, SHOOTER_ANGLE_MIN, SHOOTER_ANGLE_MAX);
    }

    public void setVelocity(double frontVelocityMPS, double rearVelocityMPS){
        targetFrontMPS = MathUtils.clamp(frontVelocityMPS, 0, MAX_MPS);
        targetRearMPS  = MathUtils.clamp(rearVelocityMPS,  0, MAX_MPS);
        myOpMode.telemetry.addData("SET CPS",   "A: %5.0f  B: %5.0f %s Angle: %.0f", frontVelocityMPS / SHOOTER_COUNTS_TO_MPS, rearVelocityMPS / SHOOTER_COUNTS_TO_MPS, atSpeed ? "At Speed" : "SLOW", tiltAngle);
    }
}
