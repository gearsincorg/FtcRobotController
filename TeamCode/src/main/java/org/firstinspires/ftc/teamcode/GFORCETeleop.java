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
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

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
    final double HEADING_HOLD_TIME  = 10.0 ; // How long (in seconds) to hold heading once all driver input stops. (This Avoids effects of Gyro Drift)
    final double IN_FRONT_ANGLE = 0.14 ;
    final double VERY_IN_FRONT_ANGLE = 0.05 ;
    final double CLICK_ON_SPEED = 0.2 ;
    final double APPROACH_SPEED = 0.3 ;
    final double STRAFE_GAIN = 1.5 ;

    private static final double YAW_GAIN            = 0.018;    // Strength of Yaw position control
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    // local parameters
    ElapsedTime stopTime   = new ElapsedTime();  // Use for timeouts.
    boolean autoHeading    = false; // used to indicate when heading should be locked.
    double heading = 0;
    double turnrate = 0;
    public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    // get an instance of the "Robot" class.
    LiftSubsystem lift = new LiftSubsystem(this);
    //VisionSubsystem camera = new VisionSubsystem(this);
    ArmSubsystem arm = new ArmSubsystem(this);
    IntakeSubsystem intake = new IntakeSubsystem(this);


    @Override public void runOpMode()
    {
        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the drive hardware & Turn on telemetry
       // camera.initilaize(true);
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

            // read joystick values and scale according to limits set at top of this file
            double drive  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;      //  Fwd/back on left stick
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;     //  Left/Right on left stick
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;       //  Rotate on right stick

            // Get the latest sensor data every time around the loop.
            lift.update();
            arm.update();


            //  OR... For special conditions, Use the DPAD to make slow-mo orthogonal motions.  Adjust the divider to your needs.
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

            //collecter test
            if (gamepad2.dpad_up){
                intake.intake();
            } else if (gamepad2.dpad_down){
                intake.eject();
            } else{
                intake.off();
            }



            /*if (gamepad1.right_trigger > 0.25) {
                double xError = 0 - camera.getTargetX();
                strafe = xError * STRAFE_GAIN;

                double yError = robot.frontRange - 2;
                if ((robot.frontRange > 5) && (Math.abs(xError) < IN_FRONT_ANGLE)) {
                    drive =APPROACH_SPEED;
                } else if ((robot.frontRange <= 5) && (Math.abs(xError) < VERY_IN_FRONT_ANGLE)) {
                    drive =CLICK_ON_SPEED;
                }

            }
 */
            // This is where we keep the robot heading locked so it doesn't turn while driving or strafing in a straight line.
            // Is the driver turning the robot, or should it hold its heading?
            heading = robot.pose.heading.toDouble();
            turnrate = robot.lazyImu.get().getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
            if (Math.abs(yaw) > 0.05) {
                // driver is commanding robot to turn, so turn off auto heading.
                autoHeading = false;
            } else {
                // If we are not already locked, wait for robot to stop rotating (<2 deg per second) and then lock-in the current heading.
                if (!autoHeading && Math.abs(turnrate) < 2.0) {
                    yawController.reset(heading);  // Lock in the current heading
                    autoHeading = true;
                }
            }

            // If auto heading is on, override manual yaw with the value generated by the heading controller.
            if (autoHeading) {
                yaw = yawController.getOutput(heading);
            }


            //  Drive the wheels based on the desired axis motions
            robot.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            drive,
                            strafe
                    ),
                    yaw
            ));

            robot.updatePoseEstimate();

            telemetry.addData("x", robot.pose.position.x);
            telemetry.addData("y", robot.pose.position.y);
            telemetry.addData("heading (deg)", heading);
            telemetry.addData("turn rate", turnrate);
            telemetry.addData("power","drive %.2f strafe %.2f yaw %.2f", drive, strafe, yaw);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


        }
    }
}