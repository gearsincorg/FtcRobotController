package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auxtools.SharedOQ;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;
import org.firstinspires.ftc.teamcode.auxtools.Target;

import static org.firstinspires.ftc.teamcode.subsystems.TurretStates.*;

import androidx.core.math.MathUtils;

public class TurretSubsystem extends SubsystemBase {

    public TurretSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private VisionSubsystem visionSubsystem;
    private DcMotorEx aim;
    private DcMotorEx shoot;
    private DcMotorEx rollers;
    private Servo hood;
    private DigitalChannel magnet;

    private Target target = new Target();

    // Subsystem Constants
    private final double COUNTS_PER_DEGREES = 145.1 * 135 / 21 / 360;
    private final double ROLLER_COUNTS_TO_MPS = 0.072 * Math.PI / 28;
    private final double SHOOTER_COUNTS_TO_MPS = 0.072 * Math.PI / 28;
    private final double RED_X =  -1828.8;
    private final double RED_Y =   1828.8;
    private final double BLUE_X = -1828.8;
    private final double BLUE_Y = -1828.8;

    private final double MIN_TURRET_ANGLE = -55;
    private final double MAX_TURRET_ANGLE =  55;
    private final double AIM_MARGIN       =   2;
    private final double TURRET_OFFSET_ANGLE = 60;

    private final double SHOOTER_STEP = 0.05;

    // General Subsystem Members
    private double shooterPower = 0.5;

    private double Aa = 0;
    private double Ar = 0;
    private double At = 0;  // measured Turret angle
    private double Ad = 0;  // desired Turret angle (assuming +/- 180 range)
    private boolean turretInPosition = false;
    private boolean turretOnTarget   = false;

    private PIDFCoefficients pidf;

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);  // do not remove
        setState(INIT);

        aim = myOpMode.hardwareMap.get(DcMotorEx.class, "aim");
        aim.setDirection(DcMotorSimple.Direction.REVERSE);
        aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidf = aim.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        aim.setPositionPIDFCoefficients(26);

        shoot = myOpMode.hardwareMap.get(DcMotorEx.class, "shooter");
        shoot.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rollers = myOpMode.hardwareMap.get(DcMotorEx.class, "rollers");
        rollers.setDirection(DcMotorSimple.Direction.REVERSE);
        rollers.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        turretInPosition = !aim.isBusy();
        turretOnTarget   = (Math.abs(Ad-At) < AIM_MARGIN);
        target = visionSubsystem.findTarget();
    }

    /**
     * Run any non-state machine pre-processing
     * Called every update() Cycle
     */
    public void runProcessing() {
        // only drive turret once it's been homed.
        if (currentState != INIT) {
            updateShooterSpeed();  // this is just here for testing.

            // If we can see the apriltag, use it to point the turret,
            // otherwise use the angle calculated from the robot's location on the field.
            //if (target.isValid) {
            //    Ad = At + target.bearing;
            //} else {
                Ad = calculateAd();
            //}
            goToTurretAd(Ad);
        }
    }

    @Override
    public void runStateMachine() {
        switch ((TurretStates) currentState) {
            case INIT: {
                aim.setPower(0.15);
                if (!magnet.getState()) {
                    // reset encoder, lock in current position and switch to RTP mode
                    aim.setPower(0.0);
                    aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    aim.setTargetPosition(aim.getCurrentPosition());
                    aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    aim.setPower(0.5);

                    goToTurretAd(0);
                    setState(HOMING);
                }
                break;
            }

            case HOMING: {
                if (!aim.isBusy()) {
                    setState(HOME);
                }
                break;
            }

            case HOME: {

                break;
            }
        }

        // Save current state in Globals for other subsystems
        Globals.TURRET_STATE = (TurretStates) currentState;
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
        myOpMode.telemetry.addData("Turret", "%s At=%4.0f, Ad=%4.0f, Aa=%4.0f, Ar=%4.0f", currentState, At, Ad, Aa, Ar);
        myOpMode.telemetry.addData("Shooter", "P=%5.2f V=%.1f ", shooterPower, shoot.getVelocity() * SHOOTER_COUNTS_TO_MPS);
        myOpMode.telemetry.addData("Roller", "P=%5.2f V=%.1f ", shooterPower, rollers.getVelocity() * ROLLER_COUNTS_TO_MPS);
        myOpMode.telemetry.addData("shooter pidf P=%f", pidf.p);
    }

    /**
     * Convert any angle to a +/- 180  degree value.
     *
     * @param angle
     * @return
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }

    private double calculateAd() {
        double targetX, targetY ;
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED) {
            targetX = RED_X;
            targetY = RED_Y;
        } else {
            targetX = BLUE_X;
            targetY = BLUE_Y;
        }

        double x = targetX - SharedOQ.OQlocalizer.posX_mm;
        double y = targetY - SharedOQ.OQlocalizer.posY_mm;

        Aa = Math.toDegrees(Math.atan2(y, x));
        Ar = Math.toDegrees(SharedOQ.OQlocalizer.heading_rad);
        return normalizeAngle(Aa - Ar);
    }

    private double encoderToDegrees(int encoder) {
        return normalizeAngle(((double)encoder / COUNTS_PER_DEGREES) + TURRET_OFFSET_ANGLE);
    }

    private int degreesToEncoder(double degrees) {
        return (int) (normalizeAngle(degrees - TURRET_OFFSET_ANGLE) * COUNTS_PER_DEGREES);
    }

    private void goToTurretAd(double Ad) {
        //  make sure the turret is kep within it's range of motion
        double clampedAd = MathUtils.clamp(Ad, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
        aim.setTargetPosition(degreesToEncoder(clampedAd));
    }
}
