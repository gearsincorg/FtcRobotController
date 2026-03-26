package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.auxtools.SharedOQ;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

import static org.firstinspires.ftc.teamcode.subsystems.TurretStates.*;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

public class TurretSubsystem extends SubsystemBase {

    private boolean TEST_MODE = false ;  //  <<---  set to true to play with shooter speed/angle
    private double CYCLE_TIME = 0.025 ;  // To anticipate motion

    public TurretSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private ShooterSubsystem shooterSubsystem = new ShooterSubsystem(myOpMode);

    private DcMotorEx aim;
    private DigitalChannel magnet;

    // Subsystem Constants
    private final double COUNTS_PER_DEGREES = 145.1 * 135 / 21 / 360;
    private final double RED_X =   400.0;
    private final double RED_Y =   0.0;
    private final double BLUE_X =  400.0;
    private final double BLUE_Y =  0.0;

    private final double MIN_TURRET_ANGLE    = -91;
    private final double MAX_TURRET_ANGLE    =  91;
    private final double AIM_MARGIN          =   2;
    private final double TURRET_OFFSET_ANGLE =  93.5;  // Adjust this if the shooter is not centered on marks at 0 deg/
    private final double TURRET_OFFSET_DISTANCE = 78;  // this is how far the Turret is from the center of the robot
    private final double AIM_PROP_GAIN       =   26;   // was 26 (30 too high?)

    private final double SHOOTER_STEP   = 2.00;
    private final double MAX_MPS        = 30;
    private final double ANGLE_STEP     = 2;

    // General Subsystem Members
    private double shooterSpeedMPS        = 0;
    private double shooterBackspinPercent = 0;
    private double shooterAngle           = 30;
    private double Ag = 0;
    private double Ar    = 0;
    private double At    = 0;  // measured Turret angle
    private double Ad    = 0;  // desired Turret angle (assuming +/- 180 range)
    private double targetRange = 0;  // Range to goal in mm

    private ElapsedTime stateTime = new ElapsedTime();

    private double[] speedCoefs = { 4.7   , 0.0022}; // C, X  was {4.5429, 0.0022};
    private double[] angleCoefs = {-3.5333, 0.0193}; // C, X

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);  // do not remove

        setState(INIT);
        aim = myOpMode.hardwareMap.get(DcMotorEx.class, "aim");
        aim.setDirection(DcMotorSimple.Direction.REVERSE);
        aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aim.setPositionPIDFCoefficients(AIM_PROP_GAIN);

        magnet = myOpMode.hardwareMap.get(DigitalChannel.class, "turret_magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);

        // initialize all the subsystem
        shooterSubsystem.init(true);
    }

    @Override
    public void update() {
        super.update();  // do not remove
        shooterSubsystem.update();
    }

    @Override
    /**
     * Read any sensor for this subsystem and calculate any derived values
     * Called every Update() cycle;
     */
    public void readSensors() {
        At = encoderToDegrees(aim.getCurrentPosition());
        Globals.TURRET_ON_TARGET = (Math.abs(Ad - At) < AIM_MARGIN);


        calculate_Ad_and_Range();
        Globals.SHOOTER_AT_SPEED = shooterSubsystem.atSpeed;
    }

    /**
     * Run any non-state machine pre-processing
     * Called every update() Cycle
     */
    public void runProcessing() {
        if ((currentState == READY) && myOpMode.opModeIsActive()) {
            //  we are in PLAY mode
            if (Globals.ROBOT_STATE == RobotStates.SHOOTING) {
                // we want to point the shooter and get wheels up to speed.
                if (TEST_MODE) {
                    // Use the gamepad to modify the shooter speed and tilt
                    setShooterManually();
                } else {
                     // calculate automatic shooter trajectory (speed and tilt)
                     solveTrajectory();
                }

                // DETERMINE and set: Turret angle, Shooter angle and individual velocities
                // Eliminate Backspin math
                if (Ad > MAX_TURRET_ANGLE || Ad < MIN_TURRET_ANGLE) {
                    Ad = normalizeAngle(Ad - 180);
                    shooterSubsystem.setAngle(-shooterAngle);
                } else {
                    shooterSubsystem.setAngle(shooterAngle);
                }
                shooterSubsystem.setVelocity(shooterSpeedMPS,shooterSpeedMPS);
                setTurretAngle(Ad);

             } else {
                // put turret in neutral position
                setTurretAngle(0);
                shooterSubsystem.setAngle(0);
            }

        } else if ((currentState == READY) && myOpMode.opModeInInit()) {
            //  we are in INIT mode

           shooterSubsystem.setVelocity(0,0);  // never run the shooter in init

            // If we are in auto, we just want to point the shooter
            if (Globals.IS_AUTO){

                // check for diagnostic home request
                if (myOpMode.gamepad1.touchpad) {
                    setTurretAngle(0);
                    shooterSubsystem.setAngle(0);
                } else {
                   solveTrajectory();

                   // DETERMINE and set: Turret angle, Shooter angle
                   if (Ad > MAX_TURRET_ANGLE || Ad < MIN_TURRET_ANGLE) {
                       Ad = normalizeAngle(Ad - 180);
                       shooterSubsystem.setAngle(-shooterAngle);
                   } else {
                       shooterSubsystem.setAngle(shooterAngle);
                   }
                   setTurretAngle(Ad);
               }
           }
        }
    }

    @Override
    public void runStateMachine() {
        switch ((TurretStates) currentState) {

            case INIT: {
                aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                aim.setPower(0.15);
                shooterSubsystem.setAngle(30);  // keep the wires out of the way while homing.
                setState(HOMING);
                break;
            }

            case HOMING: {
                if (!magnet.getState()) {
                    // reset encoder, lock in current position and switch to RTP mode
                    aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    aim.setPower(0.0);
                    aim.setTargetPosition(aim.getCurrentPosition());
                    aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    aim.setPower(0.5);
                    setTurretAngle(0);
                    setState(ALMOST_READY);
                }
                break;
            }

            case ALMOST_READY: {
                if (timeInState(0.5)){
                    setState(READY);
                }
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
        myOpMode.telemetry.addData("GOAL", "Rng=%5.0f Ag=%4.0f", targetRange, Ag);
        myOpMode.telemetry.addData("TURRET", "%s Ar=%4.0f, Ad=%4.0f, At=%4.0f\n",
                currentState, Ar, Ad, At, targetRange);
    }

    /**
     * Convert any angle to a +/- 180  degree value.
     * @param angle
     * @return
     */
    private double normalizeAngle(double angle){
        while (angle > 180) { angle -= 360; }
        while (angle < -180) { angle += 360; }
        return angle;
    }

    private void calculate_Ad_and_Range() {
        double targetX, targetY ;
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED) {
            targetX = RED_X;
            targetY = RED_Y;
        } else {
            targetX = BLUE_X;
            targetY = BLUE_Y;
        }

        // calculate robot-Turret pose with predicted motion
        double robotH = SharedOQ.OQlocalizer.heading_rad + (SharedOQ.OQlocalizer.velHeading_radS * CYCLE_TIME);
        double robotX = SharedOQ.OQlocalizer.posX_mm + (TURRET_OFFSET_DISTANCE * Math.sin(robotH)) + (SharedOQ.OQlocalizer.velX_mmS * CYCLE_TIME);
        double robotY = SharedOQ.OQlocalizer.posY_mm - (TURRET_OFFSET_DISTANCE * Math.cos(robotH)) + (SharedOQ.OQlocalizer.velY_mmS * CYCLE_TIME);

        double x = targetX - robotX;
        double y = targetY - robotY;

        targetRange = Math.hypot(x,y);
        Ag = Math.toDegrees(Math.atan2(y, x));
        Ar = Math.toDegrees(robotH);
        Ad = normalizeAngle(Ag - Ar);   
    }

    private double encoderToDegrees(int encoder) {
        return normalizeAngle(((double)encoder / COUNTS_PER_DEGREES) + TURRET_OFFSET_ANGLE);
    }

    private int degreesToEncoder(double degrees) {
        return (int) (normalizeAngle(degrees - TURRET_OFFSET_ANGLE) * COUNTS_PER_DEGREES);
    }

    private void setTurretAngle(double newAngle) {
        //  make sure the turret is kept within it's range of motion
        double clampedAd = MathUtils.clamp(newAngle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
        aim.setTargetPosition(degreesToEncoder(clampedAd));
    }

    public void solveTrajectory() {
        shooterAngle = solve(targetRange, angleCoefs);
        shooterSpeedMPS = solve(targetRange, speedCoefs) ;
    }

    /**
     * this function allows you to change the angle, speed and back spin on the turret
     * @param angle
     * @param speed
     * @param backspinPercent
     */
    public void setupShooter(double angle, double speed, double backspinPercent){
        shooterAngle = angle;
        shooterSpeedMPS = speed;
        shooterBackspinPercent = backspinPercent;
    }

    public double solve(double range, double[] coefs){
        double sum = coefs[0];
        sum += range * coefs[1];
        return sum;
    }

    // test mode
    void setShooterManually() {
        // if this is the first time through, setup initial values
        if (shooterSpeedMPS == 0) {
            setupShooter(20, 10, 0);  // Default manual settings
        }

        // DETERMINE MANUAL shooter speed
        if (myOpMode.gamepad1.dpadUpWasPressed() && (shooterSpeedMPS <= MAX_MPS)) {
            shooterSpeedMPS += SHOOTER_STEP;
        } else if (myOpMode.gamepad1.dpadDownWasPressed() && (shooterSpeedMPS >= SHOOTER_STEP)) {
            shooterSpeedMPS -= SHOOTER_STEP;
        }

        // DETERMINE MANUAL shooter TILT
        if (myOpMode.gamepad1.dpadRightWasPressed() && (shooterAngle <= shooterSubsystem.SHOOTER_ANGLE_MAX)) {
            shooterAngle += ANGLE_STEP;
        } else if (myOpMode.gamepad1.dpadLeftWasPressed() && (shooterAngle >= shooterSubsystem.SHOOTER_ANGLE_MIN)) {
            shooterAngle -= ANGLE_STEP;
        }
    }

    // =============  Action methods  ========================

    public Action actionTelemetryUpdate(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                myOpMode.telemetry.addData("ROBOT", "%s - %s", Globals.ROBOT_STATE, Globals.ALLIANCE_COLOR);
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
