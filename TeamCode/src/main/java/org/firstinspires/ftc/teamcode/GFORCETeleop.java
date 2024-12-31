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

import org.firstinspires.ftc.teamcode.subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ProportionalControl;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot using Essential Mecanum functions.
 * An external "EssentialMecanumRobot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="GFORCE Teleop", group = "AA")
public class GFORCETeleop extends LinearOpMode
{
    final double SAFE_DRIVE_SPEED   = 0.8 ; // Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED  = 0.8 ; // Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     = 0.5 ; // Slower usually means more accuracy.  Max value = 1.0

    final boolean USE_FIELD_CENTRIC_MODE = true;

    private static final double YAW_GAIN            = 0.02;    // Strength of Yaw position control 0.018
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    // local parameters
    ElapsedTime stopTime   = new ElapsedTime();  // Use for timeouts.
    boolean autoHeading    = false; // used to indicate when heading should be locked.
    boolean fieldCentric   = false;
    double headingDeg = 0;
    double turnrate = 0;
    Vector2d robotOrFieldCentric = new Vector2d(0,0);
    public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    // get an instance of each of the subsystems
    MecanumDrive    robot   ;
    LiftSubsystem lift    = new LiftSubsystem(this);
    ArmSubsystem arm     = new ArmSubsystem(this);
    IntakeSubsystem intake  = new IntakeSubsystem(this);
    AutoConfig autoConfig   = new AutoConfig(this);

    @Override public void runOpMode()
    {
        Globals.IS_AUTO = false;
        robot     = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        autoConfig.initialize();

        // Initialize the drive hardware & Turn on telemetry
        lift.initialize(true);
        arm.initialize(true);
        intake.initialize(true);

        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            robot.updatePoseEstimate();
            lift.update();
            arm.update();
            intake.update();
            telemetry.update();
        }

        // Set GLOBAL flags based on menu choices.
        if (autoConfig.autoOptions.redAlliance ){
            Globals.ALLIANCE_COLOR = AllianceColor.RED;
        } else {
            Globals.ALLIANCE_COLOR = AllianceColor.BLUE;
        }

        // Reset pose and mechanisms
        robot.setPose(Globals.LAST_POSE);  // Will be 0,0,0 if auto not run.
        lift.resetEncoders();

        Globals.OCTO_ERRORS = 0;

        while (opModeIsActive())
        {
            // Get the latest sensor data every time around the loop.
            arm.update();
            lift.update();
            intake.update();

            fieldCentric = USE_FIELD_CENTRIC_MODE;

            // Check to see if we need to home the Lift
            if (gamepad2.touchpad) {
                lift.homeTheLift();
                arm.homeTheArm();
                intake.homeTheSlide();
            }

            // update the robot's position based on the odometry pods.
            robot.updatePoseEstimate();
            Globals.LAST_POSE = robot.getPose();


            if (gamepad1.touchpad) {  // Let the driver reset the heading to one of the 4 ordinals.
                if (gamepad1.triangle) {
                    setHeadingDeg(0);
                } else if (gamepad1.circle) {
                    setHeadingDeg(-90);
                } else if (gamepad1.cross) {
                    setHeadingDeg(180);
                } else if (gamepad1.square) {
                    setHeadingDeg(90);
                }
            } else {
                if (gamepad1.triangle) { // Let the driver change the setpoint
                    yawController.reset(0);     // Facing away from driver
                } else if (gamepad1.circle) {
                    yawController.reset(-90);   // facing to the Right
                } else if (gamepad1.cross) {
                    yawController.reset(180);   // facing towards the driver
                } else if (gamepad1.square) {
                    yawController.reset(-45);   /// NON STANDARD... Facing away from Basket.
                }
            }

            // read joystick values and scale according to limits set at top of this file
            double drive  = -lessSensitive(gamepad1.left_stick_y) * SAFE_DRIVE_SPEED;      //  Fwd/back on left stick
            double strafe = -lessSensitive(gamepad1.left_stick_x) * SAFE_STRAFE_SPEED;     //  Left/Right on left stick
            double yaw    = -lessSensitive(gamepad1.right_stick_x) * SAFE_YAW_SPEED;       //  Rotate on right stick

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

            // Save the current values in globals to share with other subsystems.
            Globals.DRIVE_AXIAL   = drive;
            Globals.DRIVE_LATERAL = strafe;
            Globals.DRIVE_YAW     = yaw;

            // This is where we keep the robot heading locked so it doesn't turn while driving or strafing in a straight line.
            headingDeg = Math.toDegrees(robot.getPose().heading.toDouble());
            turnrate = robot.getTurnRateDPS();

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

            // rotate the driving commands if field centric is engaged
            robotOrFieldCentric = new Vector2d(drive, strafe);
            if (USE_FIELD_CENTRIC_MODE) {
                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                robotOrFieldCentric = new RotateVector(robotOrFieldCentric, -robot.getPose().heading.toDouble()).rotated;
            }

            //  try Rotation2d.fromDouble(angle you want to rotate).times(some Vector2d)

            //  Drive the wheels based on the desired axis motions
            robot.setDrivePowers(new PoseVelocity2d(
                    robotOrFieldCentric,
                    yaw
            ));

            telemetry.addData("x", robot.getPose().position.x);
            telemetry.addData("y", robot.getPose().position.y);
            telemetry.addData("heading (deg)", headingDeg);
            telemetry.update();

            // Update the dashboard.
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // tell AUTO to home next time
        Globals.ARM_HOMED = false;
        Globals.LIFT_HOMED = false;
        Globals.SLIDE_HOMED = false;
    }

    void setHeadingDeg(double heading) {
        robot.setPose(new Pose2d(robot.getPose().position.x, robot.getPose().position.y, Math.toRadians(heading)));
        yawController.reset(heading);
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

    public double lessSensitive(double joystick) {
        return (joystick * joystick * Math.signum(joystick));
    }
}


