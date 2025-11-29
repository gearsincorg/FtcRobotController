package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.robot.RobotState;


import org.firstinspires.ftc.teamcode.auxtools.SharedOQ;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;
import org.firstinspires.ftc.teamcode.auxtools.Target;

import static org.firstinspires.ftc.teamcode.subsystems.TurretStates.*;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

public class TurretSubsystem extends SubsystemBase {
    private final boolean TEST_MODE = false;

    public TurretSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private VisionSubsystem  visionSubsystem = new VisionSubsystem(myOpMode);
    private ShooterSubsystem shooter = new ShooterSubsystem(myOpMode);

    private DcMotorEx aim;
    private DigitalChannel magnet;
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
    private final double TURRET_OFFSET_ANGLE = 85;
    private final double TURRET_OFFSET_DISTANCE = 78;  //  78;

    private final double SHOOTER_STEP = 0.25;
    private final double MAX_MPS = 30;
    private final double ANGLE_STEP = 2;
    private final Vector2d SPEED_POINT_ONE = new Vector2d(25, 7.5);
    private final Vector2d SPEED_POINT_TWO = new Vector2d(1500, 9.5);

    private final Vector2d ANGLE_POINT_ONE = new Vector2d(25, 10);
    private final Vector2d ANGLE_POINT_TWO = new Vector2d(1500, 38);

    // General Subsystem Members
    private double shooterSpeedMPS        = 0;
    private double shooterBackspinPercent = 0;
    private double shooterAngle           = 30;
    private double Aa    = 0;
    private double Ar    = 0;
    private double At    = 0;  // measured Turret angle
    private double Ad    = 0;  // desired Turret angle (assuming +/- 180 range)
    private double range = 0;  // Range to goal in mm
    private double shooterMPS = 0.0;
    private Vector2d speedCoefs;
    private Vector2d angleCoefs;


    private boolean turretInPosition = false;

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

        speedCoefs = calculateCoefs(SPEED_POINT_ONE, SPEED_POINT_TWO);
        angleCoefs = calculateCoefs(ANGLE_POINT_ONE, ANGLE_POINT_TWO);

        // initialize all the subsystem
        // visionSubsystem.init(true);
        shooter.init(true);
    }

    @Override
    public void update() {
        super.update();  // do not remove
        shooter.update();
    }

    @Override
    /**
     * Read any sensor for this subsystem and calculate any derived values
     * Called every Update() cycle;
     */
    public void readSensors() {
        At = encoderToDegrees(aim.getCurrentPosition());
        turretInPosition = !aim.isBusy();
        Globals.TURRET_ON_TARGET = (Math.abs(Ad-At) < AIM_MARGIN);
        target = visionSubsystem.findTarget();
        Globals.SHOOTER_AT_SPEED = shooter.atSpeed;
    }

    /**
     * Run any non-state machine pre-processing
     * Called every update() Cycle
     */
    public void runProcessing() {
        if(TEST_MODE){
            if (myOpMode.gamepad1.dpadUpWasPressed()  && (shooterSpeedMPS <= MAX_MPS)) {
                shooterSpeedMPS += SHOOTER_STEP;
            }
            if (myOpMode.gamepad1.dpadDownWasPressed() && (shooterSpeedMPS >= SHOOTER_STEP)) {
                shooterSpeedMPS -= SHOOTER_STEP;
            }

            if (myOpMode.gamepad1.dpadRightWasPressed()  && (shooterAngle <= shooter.SHOOTER_ANGLE_MAX)) {
                shooterAngle += ANGLE_STEP;
            }
            if (myOpMode.gamepad1.dpadLeftWasPressed() && (shooterAngle >= shooter.SHOOTER_ANGLE_MIN)) {
                shooterAngle -= ANGLE_STEP;
            }

            shooter.setAngle(shooterAngle);
            shooter.setVelocity(shooterSpeedMPS, shooterSpeedMPS);
        } else {
            // only drive turret once it's been homed.
            if (currentState == READY && Globals.ROBOT_STATE == RobotStates.SHOOTING) {
                calculate_Ad_Range();

                // calculate speed and angle for shooter trajectory
                autoAim();

                //determine which way we are pointing and change angles to compensate
                if (Ad > MAX_TURRET_ANGLE || Ad < MIN_TURRET_ANGLE) {
                    Ad = normalizeAngle(Ad - 180);
                    shooter.setAngle(-shooterAngle);
                    shooter.setVelocity(shooterSpeedMPS * (1.0 - (shooterBackspinPercent / 100)),
                            shooterSpeedMPS * (1.0 + (shooterBackspinPercent / 100)));
                } else {
                    shooter.setAngle(shooterAngle);
                    shooter.setVelocity(shooterSpeedMPS * (1.0 + (shooterBackspinPercent / 100)),
                            shooterSpeedMPS * (1.0 - (shooterBackspinPercent / 100)));
                }
                setTurretAngle(Ad);
            }
        }
    }

    @Override
    public void runStateMachine() {
        switch ((TurretStates) currentState) {

            case INIT: {
                if (Globals.TURRET_HAS_HOMED && !Globals.IS_AUTO) {
                    setState(HOMING);
                } else {
                    aim.setPower(0.15);
                    if (!magnet.getState()) {
                        // reset encoder, lock in current position and switch to RTP mode
                        aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        setState(HOMING);
                    }
                }
                break;
            }

            case HOMING: {
                aim.setPower(0.0);
                aim.setTargetPosition(aim.getCurrentPosition());
                aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                aim.setPower(0.5);
                setTurretAngle(0);
                setState(READY);
                break;
            }

            case READY: {
                break;
            }
        }

        // Save current state in Globals for other subsystems
        Globals.TURRET_STATE = (TurretStates) currentState;
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("TURRET", "%s At=%4.1f, Ad=%4.1f, Aa=%4.1f, Ar=%4.1f, range=%.1f", currentState, At, Ad, Aa, Ar, range);
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

    private void calculate_Ad_Range() {
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

        range = Math.hypot(x,y);
        Aa = Math.toDegrees(Math.atan2(y, x));
        Ar = Math.toDegrees(SharedOQ.OQlocalizer.heading_rad);
        Ad = Aa - Ar;
    }

    private double encoderToDegrees(int encoder) {
        return normalizeAngle(((double)encoder / COUNTS_PER_DEGREES) + TURRET_OFFSET_ANGLE);
    }

    private int degreesToEncoder(double degrees) {
        return (int) (normalizeAngle(degrees - TURRET_OFFSET_ANGLE) * COUNTS_PER_DEGREES);
    }

    private void setTurretAngle(double newAngle) {
        //  make sure the turret is kep within it's range of motion
        double clampedAd = MathUtils.clamp(newAngle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
        aim.setTargetPosition(degreesToEncoder(clampedAd));
    }

    public void autoAim() {
        shooterAngle = solve(range, angleCoefs);
        shooterSpeedMPS = solve(range, speedCoefs) ;
    }

    /**
     * this function allows you to change the angle speed and back spin on the turret
     * @param angle
     * @param speed
     * @param backspinPercent
     */
    public void setupShooter(double angle, double speed, double backspinPercent){
        shooterAngle = angle;
        shooterSpeedMPS = speed;
        shooterBackspinPercent = backspinPercent;
    }

    public Vector2d calculateCoefs(Vector2d p1, Vector2d p2){
        double m = p2.y - p1.y / p2.x - p1.x;
        double c = p1.y - m * p1.x;

        return new Vector2d(m,c);
    }

    public double solve(double range, Vector2d coefs){
        return coefs.x * range + coefs.y;
    }


    // =============  Action methods  ========================

    public Action actionTelemetryUpdate(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                myOpMode.telemetry.update();
                myOpMode.telemetry.addData("ROBOT", "%s - %s", Globals.ROBOT_STATE, Globals.ALLIANCE_COLOR);
                return false;
            }
        };
    }

    public Action actionSetupShooter(double angle, double speed, double backspinPercent){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                setupShooter(angle, speed, backspinPercent);
                return false;
            }
        };
    }
}
