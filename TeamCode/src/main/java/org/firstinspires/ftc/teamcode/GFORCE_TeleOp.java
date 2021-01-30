/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Teleop opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.teamcode.RingHandler.COLLECTING;
import static org.firstinspires.ftc.teamcode.RingHandler.IDLE;
import static org.firstinspires.ftc.teamcode.RingHandler.SPIN_UP;
import static org.firstinspires.ftc.teamcode.RingHandler.STOP_COLLECT;

@TeleOp(name="G-FORCE TELEOP", group="!Competition")
public class GFORCE_TeleOp extends LinearOpMode {

    // Constants
    public final double PRECISE_AXIAL_JS_SCALE = 0.4;  // .2
    public final double PRECISE_YAW_JS_SCALE = 0.3;   // .15
    public final double SLOW_AXIAL_JS_SCALE = 0.6;  // .2
    public final double SLOW_YAW_JS_SCALE = 0.5;   // .15
    public final double SHOOTER_SPEED_INCREASE = 50;

    //Shooter Speed Management
    public double spinnerSpeed = 0;
    public boolean shooterFast = false;
    public boolean lastShooterFast = false;
    public boolean shooterSlow = false;
    public boolean lastShooterSlow = false;


    // click detector variables
    boolean collectorPressed = false;
    boolean lastCollectorPressed = false;
    boolean spinnerPressed = false;
    boolean lastSpinnerPressed = false;

    // Time Keeping
    private ElapsedTime neutralTime = new ElapsedTime();
    private ElapsedTime cycleTimer  = new ElapsedTime();
    private ElapsedTime stateTimer  = new ElapsedTime();

    /* Declare OpMode members. */
    GFORCE_Hardware robot  = new GFORCE_Hardware();
    GFORCE_Vision   vision = new GFORCE_Vision();
    RingHandler     ringState = IDLE;

    @Override
    public void runOpMode() {
        double forwardBack;
        double rotate;
        double deltaLimit;

        double axialVel;
        double yawVel;
        double lastAxialVel = 0;
        double desiredHeading = 0;
        boolean driving = true;
        boolean autoHeadingOn = false;

        /* Initialize the hardware variables.
         * The init() method of the Hardware class does all the work here
         */
        robot.init(this);
        vision.init(this);
        vision.activateVuforiaTargets(true);

        // Wait for the game to start (Driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();

        waitForStart();
        spinnerSpeed = robot.HIGH_SHOOTER_SPEED;
        robot.releaseWobbleGoal();
        robot.startMotion();

        // Run until the end of the match (Driver presses STOP)
        while (opModeIsActive()) {
            robot.updateMotion();  // Read all sensors and calculate motions
            setSpinnerSpeed();     // Set the shooter speed according to buttons
            runRingHandler();
            runWobbleGrabber();

            //Driver Controls
            if (gamepad1.back && gamepad1.start) {
                robot.resetHeading();
                desiredHeading = 0;
            }

            // Manual driving
            forwardBack = -gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;

            // Go super slow for shoot and collect, or Go sorta slow if not pressing Turbo Button
            if ((ringState == COLLECTING) || (ringState == SPIN_UP)) {
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
                }


                if (robot.LOGGING) RobotLog.ii("TARGET", String.format("H:R:T:RB:S, %.1f, %.1f, %.1f, %.1f, %.1f ",
                        robot.currentHeading,
                        vision.robotBearing,
                        vision.targetBearing,
                        vision.relativeBearing,
                        desiredHeading));

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

            // determine yaw velocity from either manual or auto control
            if (autoHeadingOn && (neutralTime.time() < 3)) {
                robot.setYawVelocityToHoldHeading(desiredHeading);
            } else {
                robot.setYawVelocity(yawVel);
                desiredHeading = robot.currentHeading;
            }

            // send outputs to drive motors
            robot.setAxialVelocity(axialVel);
            robot.moveRobotVelocity();

            // Send telemetry message to signify robot running
            robot.showEncoders();
        }

        vision.deactivateVuforiaTargets();
    }

    //Shooter Methods
    public double setSpinnerSpeed() {
        shooterFast = (gamepad1.dpad_up);
        shooterSlow = (gamepad1.dpad_down);

        // Let copilot set target speed.
        if (gamepad2.dpad_up)
            spinnerSpeed = robot.HIGH_SHOOTER_SPEED;
        else if (gamepad2.dpad_right)
            spinnerSpeed = robot.MID_SHOOTER_SPEED;
        else if (gamepad2.dpad_down)
            spinnerSpeed = robot.WOBBLE_SHOOTER_SPEED;
        else if (gamepad2.dpad_left)
            spinnerSpeed = robot.POWER_SHOT_SPEED;


        //Look for button clicks and adjust speed
        if (shooterFast && !lastShooterFast) {
            spinnerSpeed += SHOOTER_SPEED_INCREASE;
        } else if (shooterSlow && !lastShooterSlow) {
            spinnerSpeed -= SHOOTER_SPEED_INCREASE;
        }

        //Clipping so that it does not go too fast
        spinnerSpeed = Range.clip(spinnerSpeed, 0, robot.MAX_VELOCITY);

        //Set to previous values before looping again
        lastShooterFast = shooterFast;
        lastShooterSlow = shooterSlow;
        return (spinnerSpeed);
    }


    private void runRingHandler() {
        switch (ringState) {
            case IDLE:
                if (toggleCollector()) {
                    robot.runCollectors(1);
                    ringState = COLLECTING;
                } else if (toggleSpinner() || gamepad1.right_bumper) {
                    robot.runSpinners(spinnerSpeed);
                    robot.releaseRings();
                    ringState = SPIN_UP;
                } else {
                    robot.stopRings();
                    if (gamepad2.left_trigger > 0.5) {
                        robot.runCollectors(-1);
                    } else {
                        robot.runCollectors(0);
                    }
                    if (gamepad2.right_trigger > 0.5) {
                        robot.runSpinners(-spinnerSpeed);
                    } else {
                        robot.runSpinners(0);
                    }
                }
                break;

            case COLLECTING:
                if (toggleCollector()){
                    robot.runCollectors(0);
                    ringState = IDLE;
                } else if (toggleSpinner() || gamepad1.right_bumper) {
                    robot.runCollectors(0);
                    robot.runSpinners(spinnerSpeed);
                    stateTimer.reset();
                    ringState = STOP_COLLECT;
                } else {
                    if (gamepad2.left_trigger > 0.5) {
                        robot.runCollectors(-1);
                    } else {
                        robot.runCollectors(1);
                    }
                }
                break;

            case STOP_COLLECT:
                if (stateTimer.time() > 0.2) {
                    robot.releaseRings();
                    ringState = SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (toggleSpinner() || (gamepad1.right_trigger > 0.5)) {
                    robot.runSpinners(0);
                    robot.stopRings();
                    ringState = IDLE;
                } else if (toggleCollector()) {
                    robot.stopRings();
                    robot.runSpinners(0);
                    robot.runCollectors(1);
                    ringState = COLLECTING;
                } else if (robot.spinnerAtSpeed(spinnerSpeed) && (gamepad1.right_bumper) ) {
                    robot.runCollectors(1);
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

}
