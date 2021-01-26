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

@TeleOp(name="G-FORCE Teleop", group="!Competition")
public class GFORCE_TeleOp extends LinearOpMode {

    // Constants
    public final double SLOW_AXIAL_JS_SCALE = 0.2;
    public final double SLOW_YAW_JS_SCALE = 0.15;
    public final double SHOOTER_SPEED_INCREASE = 50;

    //Shooter Speed Management
    public double spinnerSpeed = 0;
    public boolean shooterFast = false;
    public boolean lastShooterFast = false;
    public boolean shooterSlow = false;
    public boolean lastShooterSlow = false;

    // Image Targeting
    public  double robotX;
    public  double robotY;
    public  double targetRange;
    public  double targetBearing;
    public  double robotBearing;
    public  double relativeBearing;
    public  OpenGLMatrix        robotLocation;

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
    GFORCE_Hardware robot = new GFORCE_Hardware();
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
        robot.activateVuforiaTargets();

        // Wait for the game to start (Driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();

        waitForStart();
        spinnerSpeed = robot.INITIAL_SHOOTER_SPEED;
        robot.grabWobbleGoal();
        robot.startMotion();

        // Run until the end of the match (Driver presses STOP)
        while (opModeIsActive()) {
            robot.updateMotion();  // Read all sensors and calculate motions
            setSpinnerSpeed();     // Set the shooter speed according to buttons
            runRingHandler();

            //Driver Controls
            if (gamepad1.back && gamepad1.start) {
                robot.resetHeading();
                desiredHeading = 0;
            }

            forwardBack = gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;

            if (gamepad1.left_trigger > 0.5) {
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
            if (gamepad1.left_bumper) {
                // Look to see if we have a new target lock....
                if (newTargetPosition()) {
                    RobotLog.ii("TARGET", "New Position");
                    autoHeadingOn = true;
                    desiredHeading = robot.currentHeading + relativeBearing;
                }
                RobotLog.ii("TARGET", String.format("H:R:T:RB:S, %.1f, %.1f, %.1f, %.1f, %.1f ",
                        robot.currentHeading,
                        robotBearing,
                        targetBearing,
                        relativeBearing,
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

    }
    //Shooter Methods
    public double setSpinnerSpeed() {
        shooterFast = (gamepad1.dpad_up);
        shooterSlow = (gamepad1.dpad_down);

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

    /***
     * Look for new target position and generate tracking data
     * @return
     */
    private boolean newTargetPosition(){

        // check all the trackable targets to see which one (if any) is visible.
        boolean newTargetFound  = false;
        robot.targetVisible     = false;
        double lrobotX;
        double lrobotY;
        double ltargetRange;
        double ltargetBearing;
        double lrobotBearing;

        for (VuforiaTrackable trackable : robot.allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                robot.targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    robot.lastLocation = robotLocationTransform;

                    robotLocation = robotLocationTransform;
                    VectorF trans = robotLocation.getTranslation();
                    Orientation rot = Orientation.getOrientation(robotLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Robot position is defined by the standard Matrix translation (x and y)
                    lrobotX = trans.get(0);
                    lrobotY = trans.get(1);

                    // Robot bearing (in cartesian system) is defined by the standard Matrix z rotation
                    lrobotBearing = rot.thirdAngle;

                    // target range is based on distance from robot position to origin.
                    ltargetRange = Math.hypot(lrobotX, lrobotY);

                    // target bearing is based on angle formed between the X axis to the target range line
                    ltargetBearing = Math.toDegrees(-Math.asin(lrobotY / ltargetRange));

                    // sanity check
                    if ((Math.abs(lrobotBearing) < 30) && (Math.abs(ltargetBearing) < 30)) {
                        robotX = lrobotX;
                        robotY = lrobotY;
                        robotBearing = lrobotBearing;
                        targetRange = ltargetRange;
                        targetBearing = ltargetBearing;

                        // Target relative bearing is the target currentHeading relative to the direction the robot is pointing.
                        relativeBearing = targetBearing - robotBearing;
                        newTargetFound = true;
                    }

                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (robot.targetVisible) {
            // express position (translation) of robot in inches.
            telemetry.addData("Robot", "X, Y (H) = %.0f, %.0f (%.1f)", robotX, robotY, robotBearing);
            telemetry.addData("Target", "R (B) (RB) = %.0f  (%.1f) (%.1f)", targetRange, targetBearing, relativeBearing);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        return(newTargetFound);
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
                    robot.runCollectors(0);
                    robot.runSpinners(0);
                    robot.stopRings();
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
}
