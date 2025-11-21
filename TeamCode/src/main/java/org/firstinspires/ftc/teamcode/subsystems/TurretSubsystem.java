package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import org.firstinspires.ftc.teamcode.auxtools.SharedOQ;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;
import org.firstinspires.ftc.teamcode.auxtools.Target;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import static org.firstinspires.ftc.teamcode.subsystems.TurretStates.*;

import androidx.core.math.MathUtils;

public class TurretSubsystem extends SubsystemBase {

    public TurretSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private VisionSubsystem visionSubsystem;
    private DcMotorEx aim;
    private DigitalChannel magnet;
    private ShooterSubsystem shooter;

    private Target target = new Target();

    // Subsystem Constants
    private final double COUNTS_PER_DEGREES = 145.1 * 135 / 21 / 360;
    private final double RED_X =  -1828.8;
    private final double RED_Y =   1828.8;
    private final double BLUE_X = -1828.8;
    private final double BLUE_Y = -1828.8;

    private final double MIN_TURRET_ANGLE = -90;
    private final double MAX_TURRET_ANGLE =  90;
    private final double AIM_MARGIN       =   2;
    private final double TURRET_OFFSET_ANGLE = 90;
    private final double TURRET_OFFSET_DISTANCE = 78;

    // General Subsystem Members

    private double Aa    = 0;
    private double Ar    = 0;
    private double At    = 0;  // measured Turret angle
    private double Ad    = 0;  // desired Turret angle (assuming +/- 180 range)
    private double Atilt = 0;
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

        magnet = myOpMode.hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);

        // initialize the vision subsystem 
        visionSubsystem = new VisionSubsystem(myOpMode);
        visionSubsystem.init(true);

        shooter = new ShooterSubsystem(myOpMode);
        shooter.init(true);
    }

    @Override
    /**
     * Read any sensor for this subsystem and calculate any derived values
     * Called every Update() cycle;
     */
    public void readSensors() {
        shooter.update();

        At = encoderToDegrees(aim.getCurrentPosition());
        turretInPosition = !aim.isBusy();
        turretOnTarget   = (Math.abs(Ad-At) < AIM_MARGIN);
        target = visionSubsystem.findTarget();
        Globals.AT_SPEED = shooter.atSpeed;
    }

    /**
     * Run any non-state machine pre-processing
     * Called every update() Cycle
     */
    public void runProcessing() {
        // only drive turret once it's been homed.

        Atilt = 40;

        if (currentState != INIT) {
            Ad = calculateAd();

            //determine which way we are pointing and change angles to compensate
            if (Ad > MAX_TURRET_ANGLE || Ad < MIN_TURRET_ANGLE){
                Ad = normalizeAngle(Ad - 180);
                shooter.setAngle(-Atilt);
            }else {
                shooter.setAngle(Atilt);
            }

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


    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("Turret", "%s At=%4.0f, Ad=%4.0f, Aa=%4.0f, Ar=%4.0f", currentState, At, Ad, Aa, Ar);
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

        double robotX = SharedOQ.OQlocalizer.posX_mm + (TURRET_OFFSET_DISTANCE * Math.sin(SharedOQ.OQlocalizer.heading_rad));
        double robotY = SharedOQ.OQlocalizer.posY_mm - (TURRET_OFFSET_DISTANCE * Math.cos(SharedOQ.OQlocalizer.heading_rad));

        double x = targetX - robotX;
        double y = targetY - robotY;

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
