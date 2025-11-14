package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auxtools.SharedOQ;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

import static org.firstinspires.ftc.teamcode.subsystems.TurretStates.*;

public class TurretSubsystem extends SubsystemBase {

    public TurretSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private VisionSubsystem visionSubsystem;
    private DcMotor aim;
    private DcMotorEx shoot;
    private DcMotorEx rollers;
    private Servo hood;
    private DigitalChannel magnet;

    // Subsystem Constants
    private final double DEADBAND = 1.0;
    private final double OUTPUT_LIMIT = 0.75;
    private final double GAIN = 0.01; // was 0.005
    private final double WARNING = 140;
    private final int ONE_ROTATION = 537;
    private final double COUNTS_PER_DEGREES = 145.1 * 135 / 21 / 360;
    private final double ROLLER_COUNTS_TO_MPS = 0.072 * Math.PI / 28;
    private final double SHOOTER_COUNTS_TO_MPS = 0.072 * Math.PI / 28;
    private final double RED_X = -1482;
    private final double RED_Y = -1413;
    private final double BLUE_X = -1482;
    private final double BLUE_Y = -1413;
    private final double SPIN_LIMIT = 180;
    private final double MIN_HOOD = 0.22;
    private final double MAX_HOOD = 0.78;
    private final double TURRET_OFFSET_ANGLE = 60;

    private final double SHOOTER_STEP = 0.05;

    // Subsystem Speed/Power constants


    // Servo positions

    // General Subsystem Members
    private double shooterPower = 0.5;

    private double error = 0;
    private boolean resetting = false;

    private double Aa = 0;
    private double Ar = 0;
    private double At = 0;
    private double Ad = 0;

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);  // do not remove
        setState(INIT);

        aim = myOpMode.hardwareMap.get(DcMotor.class, "aim");
        aim.setDirection(DcMotorSimple.Direction.REVERSE);
        aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoot = myOpMode.hardwareMap.get(DcMotorEx.class, "shooter");
        shoot.setDirection(DcMotorSimple.Direction.FORWARD);
        //shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rollers = myOpMode.hardwareMap.get(DcMotorEx.class, "rollers");
        rollers.setDirection(DcMotorSimple.Direction.REVERSE);
        //rollers.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        magnet = myOpMode.hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);

        hood = myOpMode.hardwareMap.get(Servo.class, "hood");

        // initialize the vision subsystem
        visionSubsystem = new VisionSubsystem(myOpMode);
        visionSubsystem.init(true);
    }

    @Override
    /**
     * Read any sensor for this subsystem and calculate any derived values
     * Called every Update() cycle;
     */
    public void readSensors() {
        At = encoderToDegrees(aim.getCurrentPosition());
        // read and calculate turret angle, robot heading, calculate the AprilTag Angle
        calculateAd();
    }

    /**
     * Run any non-state machine pre-processing
     * Called every update() Cycle
     */
    public void runProcessing() {
        updateShooterSpeed();  // this is just here for testing.

        visionSubsystem.update();
        double bearing = visionSubsystem.getBearing();
        double output = 0;
        error = -bearing;
        if (Math.abs(error) > DEADBAND) {
            output = (error * GAIN) - (myOpMode.gamepad1.right_stick_x * 0.15);
            output = Range.clip(output, -OUTPUT_LIMIT, OUTPUT_LIMIT);
        }
        //aim.setPower(output);

        /*
        if (myOpMode.gamepad1.rightBumperWasPressed()) {
            int targetPosition;

            if (Math.abs(At) > SPIN_LIMIT) {
                resetting = true;
                if (At > 0){
                    targetPosition = aim.getCurrentPosition() - ONE_ROTATION;
                } else {
                    targetPosition = aim.getCurrentPosition() + ONE_ROTATION;
                }

                aim.setTargetPosition(targetPosition);
                aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                aim.setPower(1.0);
            }
        }

        if (resetting){
            if (!aim.isBusy()){
                aim.setPower(0.0);
                aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                resetting = false;
            }
        } else {
        }
         */
    }

    @Override
    public void runStateMachine() {
        switch ((TurretStates) currentState) {
            case INIT: {
                aim.setPower(0.15);
                setState(HOMING);
                break;
            }

            case HOMING: {
                if (!magnet.getState()) {
                    aim.setPower(0.0);
                    aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    goToTurretAngle(0);
                    setState(HOME);
                }
                break;
            }

            case HOME: {

                break;
            }
        }
    }

    public void updateShooterSpeed() {
        if (myOpMode.gamepad1.bWasPressed() && (shooterPower <= 1)) {
            shooterPower += SHOOTER_STEP;
        }
        if (myOpMode.gamepad1.aWasPressed() && (shooterPower >= SHOOTER_STEP)) {
            shooterPower -= SHOOTER_STEP;
        }

        if (myOpMode.gamepad1.right_trigger > 0.25) {
            shoot.setPower(shooterPower);
            rollers.setPower(shooterPower);
        } else {
            shoot.setPower(0);
            rollers.setPower(0);
        }
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("Turret", "%s At=%4.0f, Ad=%4.0f, Aa=%4.0f, Ar=%4.0f", currentState, At, Ad, Aa, Ad);
        myOpMode.telemetry.addData("Shooter", "P=%5.2f V=%.1f ", shooterPower, shoot.getVelocity() * SHOOTER_COUNTS_TO_MPS);
        myOpMode.telemetry.addData("Roller", "P=%5.2f V=%.1f ", shooterPower, rollers.getVelocity() * ROLLER_COUNTS_TO_MPS);
    }

    /**
     * Convert any angle to a +/- 180  degree value.
     *
     * @param angle
     * @return
     */
    double normalizeAngle(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }

    private void calculateAd() {
        double x = RED_X - SharedOQ.OQlocalizer.posX_mm;
        double y = RED_Y - SharedOQ.OQlocalizer.posY_mm;
        Aa = Math.atan2(y, x);
        Ar = Math.toDegrees(SharedOQ.OQlocalizer.heading_rad);
        Ad = Aa - Ar;
    }

    private double encoderToDegrees(int encoder) {
        return normalizeAngle(((double)encoder / COUNTS_PER_DEGREES) + TURRET_OFFSET_ANGLE);
    }

    private int degreesToEncoder(double degrees) {
        return (int) (normalizeAngle(degrees - TURRET_OFFSET_ANGLE) * COUNTS_PER_DEGREES);
    }

    private void goToTurretAngle(double angle) {
        aim.setTargetPosition(degreesToEncoder(angle));
        aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aim.setPower(1.0);

        while (!myOpMode.isStopRequested() && aim.isBusy()) {
        }

        aim.setPower(0.0);
        aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
