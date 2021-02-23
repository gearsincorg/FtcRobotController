/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Teleop opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import static org.firstinspires.ftc.teamcode.RingHandler.COLLECTING;
import static org.firstinspires.ftc.teamcode.RingHandler.IDLE;
import static org.firstinspires.ftc.teamcode.RingHandler.SPUN_UP;
import static org.firstinspires.ftc.teamcode.RingHandler.STOP_COLLECT;
import static org.firstinspires.ftc.teamcode.RingHandler.WOBBLE_LOADING;

@TeleOp(name="G-FORCE TELEOP", group="!Competition")
public class GFORCE_TeleOp extends LinearOpMode {

    // Constants
    public final double PRECISE_AXIAL_JS_SCALE  =  0.30;   // .2
    public final double PRECISE_YAW_JS_SCALE    =  0.30;   // .15
    public final double SLOW_AXIAL_JS_SCALE     =  0.65;   // .2
    public final double SLOW_YAW_JS_SCALE       =  0.65;   // .15

    //Shooter Speed button Management
    public boolean shooterFast = false;
    public boolean lastShooterFast = false;
    public boolean shooterSlow = false;
    public boolean lastShooterSlow = false;

    private final double EJECT_VELOCITY = 300 ;

    // click detector variables
    private boolean collectorPressed = false;
    private boolean lastCollectorPressed = false;
    private boolean spinnerPressed = false;
    private boolean lastSpinnerPressed = false;
    private boolean fiveLeftPressed = false;
    private boolean lastFiveLeftPressed = false;
    private boolean fiveRightPressed = false;
    private boolean lastFiveRightPressed = false;

    double  desiredHeading = 0;
    boolean autoHeadingOn = false;

    // Time Keeping
    private ElapsedTime neutralTime = new ElapsedTime();
    private ElapsedTime cycleTimer  = new ElapsedTime();
    private ElapsedTime stateTimer  = new ElapsedTime();

    /* Declare OpMode members. */
    GFORCE_Hardware robot  = new GFORCE_Hardware();
    public GFORCE_Vision   vision = new GFORCE_Vision();
    RingHandler     ringState = IDLE;

    @Override
    public void runOpMode() {
        double forwardBack;
        double rotate;
        double deltaLimit;

        double axialVel;
        double yawVel;
        double lastAxialVel = 0;

        boolean driving = true;

        /* Initialize the hardware variables.
         * The init() method of the Hardware class does all the work here
         */
        robot.init(this, vision);
        vision.init(this, robot);
        vision.activateVuforiaTargets(true);

        // Wait for the game to start (Driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();

        waitForStart();
        robot.releaseWobbleGoal();
        robot.startMotion();
        robot.stopSpinners();
        robot.setSpinnerTarget(Target.HIGH_GOAL);

        // Run until the end of the match (Driver presses STOP)
        while (opModeIsActive()) {
            robot.updateMotion();  // Read all sensors and calculate motions
            setSpinnerSpeed();     // Set the shooter speed according to buttons
            runWobbleGrabber();    // Mamage the wobble goal grabber


            if (gamepad2.right_stick_button) {
                robot.tiltShot.setTargetPosition(400);
                robot.tiltShot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.tiltShot.setPower(0.40);
            } else {
                robot.tiltShot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.tiltShot.setPower(0.00);
            }

            //Driver Controls
            if (gamepad1.back && gamepad1.start) {
                robot.resetHeading();
                desiredHeading = 0;
            }


            // Manual driving
            forwardBack = -gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;

            // Go super slow for shoot and collect, or Go sorta slow if not pressing Turbo Button
            if ((ringState == COLLECTING) || (ringState == SPUN_UP)) {
                forwardBack *= PRECISE_AXIAL_JS_SCALE;
                rotate *= PRECISE_YAW_JS_SCALE;
            } else if (gamepad1.left_trigger < 0.5) {
                forwardBack *= SLOW_AXIAL_JS_SCALE;
                rotate *= SLOW_YAW_JS_SCALE;
            }

            //Scale velocities to mm per second
            axialVel = forwardBack * robot.MAX_AXIAL_MMPS;
            yawVel   = rotate      * robot.MAX_YAW_MMPS;

            // Implement Acceleration limits.
            deltaLimit = cycleTimer.time() * robot.ACCELERATION_LIMIT;

            if (Math.abs(axialVel - lastAxialVel) > deltaLimit) {
                axialVel = lastAxialVel + ((axialVel > lastAxialVel ) ? deltaLimit : -deltaLimit);
            }
            lastAxialVel = axialVel;
            cycleTimer.reset();


            // Control Yaw, using manual or auto correction or target tracking
            boolean newTarget = vision.newTargetPosition();
            if (gamepad1.left_bumper) {
                // Look to see if we have a new target lock....
                if (newTarget) {
                    RobotLog.ii("TARGET", "New Position");
                    autoHeadingOn = true;
                    desiredHeading = robot.currentHeading + vision.relativeBearing;

                    // Adjust speed of spinner based on range
                    robot.setSpinnersByRange(vision.targetRange);
                }

                if (robot.LOGGING) RobotLog.ii("TARGET", String.format("H:R:T:RB:S, %.1f, %.1f, %.1f, %.1f, %.1f ",
                        robot.currentHeading, vision.robotBearing, vision.targetBearing, vision.relativeBearing, desiredHeading));

                neutralTime.reset();
            } else {
                if (rotate != 0) {
                    // We are turning with the joystick
                    autoHeadingOn = false;
                } else if (!autoHeadingOn && robot.notTurning()) {
                    // We have just stopped turning, so lock in current heading
                    desiredHeading = robot.currentHeading;
                    autoHeadingOn = true;
                }
            }

            // Disable correction if JS are neutral for more than 3 seconds
            driving = ((forwardBack != 0) || (rotate != 0));
            if (driving)
                neutralTime.reset();

            jogHeading();          // change heading setpoint with X B buttons

            // determine yaw velocity from either manual or auto control
            if (autoHeadingOn && (neutralTime.time() < 3)) {
                robot.setYawVelocityToHoldHeading(desiredHeading);
            } else {
                robot.setYawVelocity(yawVel);
                desiredHeading = robot.currentHeading;
            }

            // send outputs to drive motors
            robot.setAxialVelocity(axialVel);

            runRingHandler();      // Manage the conveyor and spinner
            robot.moveRobotVelocity();

            // Send telemetry message to signify robot running
            robot.showEncoders();
        }

        vision.deactivateVuforiaTargets();
    }

    //Shooter Methods
    public void setSpinnerSpeed() {
        shooterFast = (gamepad1.dpad_up);
        shooterSlow = (gamepad1.dpad_down);

        // Let copilot set target speed.
        if (gamepad2.dpad_up)
            robot.setSpinnerTarget(Target.HIGH_GOAL);
        else if (gamepad2.dpad_right)
            robot.setSpinnerTarget(Target.MID_GOAL);
        else if (gamepad2.dpad_down)
            robot.setSpinnerTarget(Target.WOBBLE_GOAL);
        else if (gamepad2.dpad_left)
            robot.setSpinnerTarget(Target.CENTER_POWER_SHOT);


        //Look for button clicks and adjust speed
        if (shooterFast && !lastShooterFast) {
            robot.jogSpinnerUp();
        } else if (shooterSlow && !lastShooterSlow) {
            robot.jogSpinnerDown();
        }

        //Set to previous values before looping again
        lastShooterFast = shooterFast;
        lastShooterSlow = shooterSlow;
    }

    /* #####   Ring Handler State Machine   #####
    // Use single click to transition between Collecting and Shooting
    // Ensure that the collectors are stopped before releasing the Ring Stop
    // Control the Ring Feed speed to get consistent insertion.
    // States:
    //  IDLE            Normal driving, look for shooter or collector reverse requests
    //  COLLECTING      Collectors running, Ring Stop engaged
    //  STOP_COLLECT    Transitioning to Shoot. Wait for collector to stop turning
    //  SPUN_UP         Spinners running, Ring Stop lifted to enable shooting
    //  WOBBLE_LOADING  RingDrop engaged, Wobble Goals held, Spinners at slow, transport enabled
    */
    private void runRingHandler() {
        switch (ringState) {
            case IDLE:
                if (toggleCollector()) {                                // Transition to COLLECTING
                    robot.runCollectors(1);
                    ringState = COLLECTING;
                } else if (toggleSpinner() || gamepad1.right_bumper) {  // Transition to SPIN_UP
                    robot.runSpinners();
                    robot.turnOnTiltPID();
                    robot.releaseRings();
                    ringState = SPUN_UP;
                } else if (gamepad2.b) {                                // Transition to WOBBLE_LOADING
                    robot.enableRingDrop();
                    ringState = WOBBLE_LOADING;
                } else {                                                // Manage IDLE Activities
                    robot.stopRings();
                    if (gamepad2.left_trigger > 0.5) {
                        robot.runCollectors(-1);
                        robot.setAxialVelocity(EJECT_VELOCITY);
                    } else {
                        robot.runCollectors(0);
                    }
                    if (gamepad2.right_trigger > 0.5) {
                        robot.reverseSpinners();
                    } else {
                        robot.stopSpinners();
                    }
                }
                break;

            case COLLECTING:
                if (toggleCollector()){                                 // Transition to IDLE
                    robot.runCollectors(0);
                    ringState = IDLE;
                } else if (toggleSpinner() || gamepad1.right_bumper) {  // Transition to STOP_COLLECT
                    robot.runCollectors(0);
                    robot.turnOnTiltPID();
                    robot.runSpinners();
                    stateTimer.reset();
                    ringState = STOP_COLLECT;
                } else if (gamepad2.b) {                                // Transition to WOBBLE_LOADING
                    robot.enableRingDrop();
                    ringState = WOBBLE_LOADING;
                } else {                                                // Manage COLLECTING Activities
                    if (gamepad2.left_trigger > 0.5) {
                        robot.runCollectors(-1);
                        robot.setAxialVelocity(EJECT_VELOCITY);
                    } else if (stateTimer.time() > 0.2) {
                        robot.runCollectors(1);  //  Run after giving Stop time to come down.
                    } else {
                        robot.runCollectors(0);
                    }
                }
                break;

            case STOP_COLLECT:
                if (stateTimer.time() > 0.2) {                          // Transition to SPIN_UP
                    robot.releaseRings();
                    ringState = SPUN_UP;
                }
                break;

            case WOBBLE_LOADING:
                if (gamepad2.x) {                                       // Transition to IDLE
                    robot.disableRingDrop();
                    ringState = IDLE;
                } else if (robot.spinnerAtSpeed() && (gamepad1.right_bumper || gamepad2.left_bumper)) {
                    robot.runCollectors(1);
                } else {
                    robot.runCollectors(0);
                }
                break;

            case SPUN_UP:
                if (toggleSpinner() || (gamepad1.right_trigger > 0.5)) { // Transition to IDLE
                    robot.stopSpinners();
                    robot.turnOffTiltPID();
                    robot.stopRings();
                    ringState = IDLE;
                } else if (toggleCollector()) {                         // Transition to COLLECTING
                    robot.stopSpinners();
                    robot.turnOffTiltPID();
                    stateTimer.reset();  // enable time to stop shooter & engage ring stop.
                    ringState = COLLECTING;
                } else if (gamepad2.b) {                                // Transition to WOBBLE_LOADING
                    robot.enableRingDrop();
                    ringState = WOBBLE_LOADING;
                } else if (robot.spinnerAtSpeed() && (gamepad1.right_bumper) ) {
                    robot.runShooterFeeder();  // vary speed based on presence of ring.
                } else {
                    robot.runCollectors(0);
                }
                break;

            default:
                break;

        }
    }

    private boolean toggleCollector() {
        collectorPressed = gamepad2.left_bumper;
        boolean clicked = (collectorPressed && !lastCollectorPressed);
        lastCollectorPressed = collectorPressed;
        return clicked;
    }

    private boolean toggleSpinner() {
        spinnerPressed = gamepad2.right_bumper;
        boolean clicked = (spinnerPressed && !lastSpinnerPressed);
        lastSpinnerPressed = spinnerPressed;
        return clicked;
    }

    private void runWobbleGrabber() {
        if (gamepad2.y)
            robot.grabWobbleGoal();
        else if (gamepad2.a)
            robot.releaseWobbleGoal();
    }

    private void jogHeading() {
        fiveLeftPressed = gamepad1.x;
        fiveRightPressed = gamepad1.b;

        if (fiveLeftPressed && !lastFiveLeftPressed) {
            neutralTime.reset();
            autoHeadingOn = true;
            desiredHeading  += 5.0;
        }

        if (fiveRightPressed && !lastFiveRightPressed) {
            neutralTime.reset();
            autoHeadingOn = true;
            desiredHeading  -= 5.0;
        }

        lastFiveLeftPressed = fiveLeftPressed;
        lastFiveRightPressed = fiveRightPressed;
    }

}
