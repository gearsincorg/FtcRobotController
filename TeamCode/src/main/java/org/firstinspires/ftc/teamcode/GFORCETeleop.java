/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot using Essential Mecanum functions.
 * An external "EssentialMecanumRobot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="GFORCE Teleop", group = "AA")
public class GFORCETeleop extends LinearOpMode
{
    final double SAFE_DRIVE_SPEED   = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED  = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     = 0.5 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double IN_FRONT_ANGLE = 0.14 ;
    final double VERY_IN_FRONT_ANGLE = 0.05 ;
    final double CLICK_ON_SPEED = 0.2 ;
    final double APPROACH_SPEED = 0.3 ;
    final double STRAFE_GAIN = 1.5 ;

    final boolean USE_FIELD_CENTRIC_MODE = true;

    private static final double YAW_GAIN            = 0.02;    // Strength of Yaw position control 0.018
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    // local parameters
    ElapsedTime stopTime   = new ElapsedTime();  // Use for timeouts.
    boolean autoHeading    = false; // used to indicate when heading should be locked.
    double headingDeg = 0;
    double turnrate = 0;
    Vector2d translate = new Vector2d(0,0);
    public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    // get an instance of each of the subsystems
    MecanumDrive    robot   = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    OctoQuadIF      octoQuad= new OctoQuadIF(this);
    LiftSubsystem   lift    = new LiftSubsystem(this);
    VisionSubsystem vision  = new VisionSubsystem(this);
    ArmSubsystem    arm     = new ArmSubsystem(this);
    IntakeSubsystem intake  = new IntakeSubsystem(this);

    @Override public void runOpMode()
    {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the drive hardware & Turn on telemetry
        octoQuad.initialize(true);
        vision.initilaize(true);
        lift.initialize(true);
        arm.initialize(true);
        intake.initialize(true);

        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            lift.update();
            arm.update();
            telemetry.update();
        }

        lift.resetEncoders();

        while (opModeIsActive())
        {
            // Get the latest sensor data every time around the loop.
            lift.update();
            arm.update();

            // update the robot's position based on the odometry pods.
            robot.updatePoseEstimate();

            // Let the driver reset the heading to one of the 4 ordinals.
            if (gamepad1.touchpad) {
                if (gamepad1.y) {
                    setHeadingDeg(90);
                } else if (gamepad1.b) {
                    setHeadingDeg(0);
                } else if (gamepad1.a) {
                    setHeadingDeg(-90);
                } else if (gamepad1.x) {
                    setHeadingDeg(180);
                }
            }

            // read joystick values and scale according to limits set at top of this file
            double drive  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;      //  Fwd/back on left stick
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;     //  Left/Right on left stick
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;       //  Rotate on right stick

            //  For special conditions, Use the DPAD to make slow-mo orthogonal motions.  Adjust the divider to your needs.
            if (gamepad1.dpad_left) {
                strafe = SAFE_DRIVE_SPEED / 4.0;
            } else if (gamepad1.dpad_right) {
                strafe = -SAFE_DRIVE_SPEED / 4.0;
            } else if (gamepad1.dpad_up) {
                drive = SAFE_DRIVE_SPEED / 4.0;
            } else if (gamepad1.dpad_down) {
                drive = -SAFE_STRAFE_SPEED / 4.0;
            }

            if (gamepad2.left_trigger > 0.5){
                intake.wristIn();
            } else if (gamepad2.left_bumper){
                intake.wristOut();
            }

            if (gamepad2.right_trigger > 0.5){
                intake.in();
            } else if (gamepad2.right_bumper){
                intake.out();
            }

            if (gamepad2.start){
                lift.sampleInBucket();
            }

            //  collecter test
            if (gamepad2.dpad_up){
                intake.intake();
            } else if (gamepad2.dpad_down){
                intake.eject();
            } else{
                intake.off();
            }

            // Implement Auto Specimen Tracking
            // Center on Specimen and approach quickly at first and then slow down.
            if (gamepad1.right_trigger > 0.25) {
                ColorTarget target = vision.getTarget();
                if (target.valid) {
                    double xError = target.x;
                    strafe = xError * STRAFE_GAIN;

                    double yError = octoQuad.getBackRangeInches();
                    if ((yError > 5) && (Math.abs(xError) < IN_FRONT_ANGLE)) {
                        drive = APPROACH_SPEED;
                    } else if ((yError <= 5) && (Math.abs(xError) < VERY_IN_FRONT_ANGLE)) {
                        drive = CLICK_ON_SPEED;
                    }
                }
            }

            // This is where we keep the robot heading locked so it doesn't turn while driving or strafing in a straight line.
            headingDeg = Math.toDegrees(robot.pose.heading.toDouble());
            turnrate = robot.lazyImu.get().getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            // Is the driver turning the robot, or should it hold its heading?
            if (Math.abs(yaw) > 0.05) {
                // driver is commanding robot to turn, so turn off auto heading.
                autoHeading = false;
            } else {
                // If we are not already locked, wait for robot to stop rotating (<2 deg per second) and then lock-in the current heading.
                if (!autoHeading && Math.abs(turnrate) < 2.0) {
                    yawController.reset(headingDeg);  // Lock in the current heading
                    autoHeading = true;
                }
            }

            // If auto heading is on, override manual yaw with the value generated by the heading controller.
            if (autoHeading) {
                yaw = yawController.getOutput(headingDeg);
            }

            translate = new Vector2d(drive, strafe);

            // rotate the driving commands if field centric is engaged
            if (USE_FIELD_CENTRIC_MODE) {
                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                translate = new RotateVector(translate, -robot.pose.heading.toDouble()).rotated;
            }

            //  Drive the wheels based on the desired axis motions
            robot.setDrivePowers(new PoseVelocity2d(
                    translate,
                    yaw
            ));

            telemetry.addData("x", robot.pose.position.x);
            telemetry.addData("y", robot.pose.position.y);
            telemetry.addData("heading (deg)", headingDeg);
            //telemetry.addData("turn rate", turnrate);
            //telemetry.addData("power","drive %.2f strafe %.2f yaw %.2f", drive, strafe, yaw);
            telemetry.update();

            // Update the dashboard.
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    void setHeadingDeg(double heading) {
        robot.pose = new Pose2d(robot.pose.position.x, robot.pose.position.y, Math.toRadians(heading));
    }

    // worker class to rotate vectors
    class RotateVector {
        Vector2d rotated;
        double x;
        double y;

        // Constructor
        public RotateVector(Vector2d vector, double angleR) {
            x = vector.x * Math.cos(angleR) - vector.y * Math.sin(angleR);
            y = vector.x * Math.sin(angleR) + vector.y * Math.cos(angleR);
            rotated = new Vector2d(x,y);
        }
    }
}


