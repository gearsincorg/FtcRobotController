package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private final double SHOOTER_ANGLE_MAX       =  40.0;
    private final double SHOOTER_ANGLE_MIN       = -40.0;
    private final double PULSE_SCALE_FACTOR      =  1.8e-3;   // make this match the spindexer in 2 places once servo is reprogrammed
    private final double SHOOTER_SPEED_TOLERANCE =  1.0;
    private final double IDLE_MPS                =  2.0;
    private final double MAX_MPS                 = 16.0;

    // Subsystem Speed/Power constants
    private final double SHOOTER_STEP = 1.0;

    // Servo positions

    // General Subsystem Members
    private double  shooterMPS        = 10.0;
    private double  shooterServoValue = 0;
    private double  currentFrontMPS;
    private double  currentRearMPS;
    private double  targetFrontMPS;
    private double  targetRearMPS;

    public boolean  atSpeed = false;

    public ShooterSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);  // do not remove

        front = myOpMode.hardwareMap.get(DcMotorEx.class, "frontWheel");
        front.setDirection(DcMotorSimple.Direction.FORWARD);
        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rear = myOpMode.hardwareMap.get(DcMotorEx.class, "rearWheel");
        rear.setDirection(DcMotorSimple.Direction.REVERSE);
        rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hood = myOpMode.hardwareMap.get(Servo.class, "hood");

        setAngle(0.0);

        setState(ShooterStates.SPEEDING_UP);
    }

    @Override
    public void readSensors() {
        currentFrontMPS = front.getVelocity()   * SHOOTER_COUNTS_TO_MPS;
        currentRearMPS  = rear.getVelocity() * SHOOTER_COUNTS_TO_MPS;
    }

    public void updateShooterSpeed() {
        if (myOpMode.gamepad1.bWasPressed()  && (shooterMPS <= MAX_MPS)) {
            shooterMPS += SHOOTER_STEP;
        }
        if (myOpMode.gamepad1.aWasPressed() && (shooterMPS >= SHOOTER_STEP)) {
            shooterMPS -= SHOOTER_STEP;
        }

        if (myOpMode.opModeIsActive()) {
            if ((Globals.ROBOT_STATE == RobotStates.SHOOTING) || (myOpMode.gamepad1.right_trigger > 0.25)) {
                setVelocity(shooterMPS, shooterMPS);
            } else {
                setVelocity(IDLE_MPS, IDLE_MPS);
            }
        } else {
            setVelocity(0, 0);
        }
    }

    @Override
    public void runProcessing() {
        updateShooterSpeed();  // this is just here for testing.
        atSpeed = ((Math.abs(targetFrontMPS - currentFrontMPS) < SHOOTER_SPEED_TOLERANCE) &&
                (Math.abs(targetRearMPS - currentRearMPS) < SHOOTER_SPEED_TOLERANCE));
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("SHOOTER",   "A: %5.2f  B: %.1f %s", currentFrontMPS, currentRearMPS, atSpeed ? "At Speed" : "SLOW");
    }

    public void setAngle(double angle){
        angle = MathUtils.clamp(angle, SHOOTER_ANGLE_MIN, SHOOTER_ANGLE_MAX);
        shooterServoValue = MathUtils.clamp(0.5 - (angle * PULSE_SCALE_FACTOR / SERVO_GEAR_RATIO), 0.22, 0.78);  // make this match the spindexer in 2 places once servo is reprogrammed
        hood.setPosition(shooterServoValue);
    }

    public void setVelocity(double frontVelocityMPS, double rearVelocityMPS){
        targetFrontMPS = frontVelocityMPS;
        targetRearMPS = rearVelocityMPS;

        front.setVelocity(targetFrontMPS /  SHOOTER_COUNTS_TO_MPS);
        rear.setVelocity(targetRearMPS / SHOOTER_COUNTS_TO_MPS);
    }
}
