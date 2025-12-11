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
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auxtools.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretStates;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot using Essential Mecanum functions.
 * An external "EssentialMecanumRobot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="GFORCE Teleop", group = "AA")
public class GFORCETeleop extends LinearOpMode
{
    private static Vector2d HOME_CALIBRATION = new Vector2d(38, 33);


    // get an instance of each of the subsystems
    AutoConfig autoConfig   = new AutoConfig(this);

    // Declare OpMode members.cvzx
    private DriveSubsystem     driveSubsystem     = new DriveSubsystem(this);
    private SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem(this);
    private TurretSubsystem    turretSubsystem    = new TurretSubsystem(this);

    private ElapsedTime cycleTimer = new ElapsedTime();
    private double avgCycle = 0;
    private double lastCycle = 0;
    private double sampleCount = 0;

    @Override public void runOpMode()
    {
        Globals.IS_AUTO = false;
        Globals.OCTO_ERRORS = 0;
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
        driveSubsystem.init(null,true);
        spindexerSubsystem.init(true);
        turretSubsystem.init(true);


        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData("ROBOT", "%s - %s\n", Globals.ROBOT_STATE, Globals.ALLIANCE_COLOR);

            // Read and display sensor data
            driveSubsystem.updatePoseEstimate();
            // spindexerSubsystem.update();            //  INHIBIT any motion during TELEOP INIT
            // turretSubsystem.update();               //  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            showCycleTime();
            telemetry.update();
        }

        spindexerSubsystem.startIntaking();

        while (opModeIsActive())
        {
            telemetry.addData("ROBOT", "%s - %s\n", Globals.ROBOT_STATE, Globals.ALLIANCE_COLOR);

            // Check to see if we need to home the subsystems
            // B button assumes driving forward into wall. X Button assumes backing into wall
            if (gamepad1.back || gamepad1.touchpad) {
                if (Globals.ALLIANCE_COLOR == AllianceColor.BLUE){
                    if (gamepad1.a) {
                        homeRobot(HOME_CALIBRATION, 90);
                    } else if (gamepad1.b) {
                        homeRobot(HOME_CALIBRATION, 180);
                    } else if (gamepad1.x) {
                        homeRobot(HOME_CALIBRATION,  0);
                    }else if (gamepad1.y) {
                        homeRobot(HOME_CALIBRATION, -90);
                    }
                } else {
                    if (gamepad1.a) {
                        homeRobot(HOME_CALIBRATION, -90);
                    } else if (gamepad1.b) {
                        homeRobot(HOME_CALIBRATION, 0);
                    } else if (gamepad1.x) {
                        homeRobot(HOME_CALIBRATION, 180);
                    } else if (gamepad1.y) {
                        homeRobot(HOME_CALIBRATION, 90);
                    }
                }
            }

            // update the robot's position based on the odometry pods.
            driveSubsystem.updatePoseEstimate();
            spindexerSubsystem.update();
            turretSubsystem.update();

            // use the smart manual drive feature of the DriveSubsystem
            driveSubsystem.smartDrive();
            showCycleTime();
            telemetry.update();

            // Update the dashboard.
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), driveSubsystem.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // tell AUTO or TELEOP to home next time they run
        Globals.TURRET_HAS_HOMED = false;
    }

    private void homeRobot(Vector2d homePosition, double headingDeg) {
        Pose2d newHome;

        if (Globals.ALLIANCE_COLOR == AllianceColor.RED){
            newHome = new Pose2d(homePosition.x, -homePosition.y, Math.toRadians(headingDeg));
        } else {
            newHome = new Pose2d(homePosition.x, homePosition.y, Math.toRadians(headingDeg));
        }
        driveSubsystem.setPose(newHome);

        // Also home the turret.
        if (turretSubsystem.currentState == TurretStates.READY) {
            turretSubsystem.setState(TurretStates.INIT);
        }
    }

    private void showCycleTime() {
        if ((++sampleCount % 10) == 0) {
            double now = cycleTimer.time();
            avgCycle = (now - lastCycle) * 100;
            lastCycle = now;
        }
        telemetry.addData("Cycle Time", "%.1f mS", avgCycle);
    }
}


