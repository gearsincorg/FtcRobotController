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
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ProportionalControl;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot using Essential Mecanum functions.
 * An external "EssentialMecanumRobot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="GFORCE Teleop", group = "AA")
public class GFORCETeleop extends LinearOpMode
{
    final double SAFE_DRIVE_SPEED   = 0.9 ; // Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     = 0.6 ; // Slower usually means more accuracy.  Max value = 1.0

    private static final double YAW_GAIN            = 0.02;    // Strength of Yaw position control 0.018
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous
    private static final double STOPPED_RATE        = 0.5;     // degrees per second rate to lock in current heading

    // local parameters
    ElapsedTime stopTime   = new ElapsedTime();  // Use for timeouts.
    boolean autoHeading    = false; // used to indicate when heading should be locked.
    boolean fieldCentric   = false;
    double headingDeg = 0;
    double turnrate = 0;
    Vector2d robotOrFieldCentric = new Vector2d(0,0);
    public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    // Private Members

    // get an instance of each of the subsystems
    //DriveSubsystem robot   ;
    AutoConfig autoConfig   = new AutoConfig(this);

    // Declare OpMode members.
    private DriveSubsystem     driveSubsystem     = new DriveSubsystem(this);
    private TurretSubsystem    turretSubsystem    = new TurretSubsystem(this);
    private SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem(this);
    private IntakeSubsystem    intakeSubsystem    = new IntakeSubsystem(this);


    @Override public void runOpMode()
    {
        Globals.IS_AUTO = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        autoConfig.initialize();

        // Set GLOBAL flags based on menu choices.
        if (autoConfig.autoOptions.redAlliance ){
            Globals.ALLIANCE_COLOR = AllianceColor.RED;
        } else {
            Globals.ALLIANCE_COLOR = AllianceColor.BLUE;
        }

        // Initialize the drive hardware & Turn on telemetry
        driveSubsystem.init(new Pose2d(0, 0, 0),true);
        turretSubsystem.init(true);
        spindexerSubsystem.init(true);
        intakeSubsystem.init(true);

        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            driveSubsystem.updatePoseEstimate();
            spindexerSubsystem.update();
            telemetry.update();
        }

        // Reset pose and mechanisms
        driveSubsystem.setPose(Globals.LAST_POSE);  // Will be 0,0,0 if auto not run.
        Globals.OCTO_ERRORS = 0;

        while (opModeIsActive())
        {
            // Get the latest sensor data every time around the loop.

            // Check to see if we need to home the subsystems
            if (gamepad2.touchpad  || gamepad2.back) {

            }

            // update the robot's position based on the odometry pods.
            driveSubsystem.updatePoseEstimate();
            spindexerSubsystem.update();

            // use the smart manual drive feature of the DriveSubsystem
            driveSubsystem.smartDrive();
            telemetry.update();

            // Update the dashboard.
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), driveSubsystem.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // tell AUTO to home next time
    }

    /**
     * Set a new heading for the robot in degrees
     * @param heading
     */
    void setHeadingDeg(double heading) {
        driveSubsystem.setPose(new Pose2d(driveSubsystem.getPose().position.x, driveSubsystem.getPose().position.y, Math.toRadians(heading)));
        yawController.reset(heading);
    }

}


