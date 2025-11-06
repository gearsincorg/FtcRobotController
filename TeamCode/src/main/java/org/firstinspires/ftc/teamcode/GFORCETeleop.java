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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auxtools.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot using Essential Mecanum functions.
 * An external "EssentialMecanumRobot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="GFORCE Teleop", group = "AA")
public class GFORCETeleop extends LinearOpMode
{
    // get an instance of each of the subsystems
    AutoConfig autoConfig   = new AutoConfig(this);

    // Declare OpMode members.
    private DriveSubsystem     driveSubsystem     = new DriveSubsystem(this);
    private SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem(this);
    private TurretSubsystem    turretSubsystem    = new TurretSubsystem(this);
    private IntakeSubsystem    intakeSubsystem    = new IntakeSubsystem(this);

    @Override public void runOpMode()
    {
        Globals.IS_AUTO = false;
        telemetry.setMsTransmissionInterval(50);

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
        spindexerSubsystem.init(true);
        //turretSubsystem.init(true);
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
        spindexerSubsystem.resetEncoder();
        driveSubsystem.setPose(Globals.LAST_POSE);  // Will be 0,0,0 if auto not run.
        Globals.OCTO_ERRORS = 0;

        while (opModeIsActive())
        {
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
}


