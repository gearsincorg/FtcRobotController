/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDMode;
import org.firstinspires.ftc.teamcode.subsystems.LoggingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PrismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretStates;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name="GFORCE Teleop", group = "AA")
public class GFORCETeleop extends LinearOpMode
{
    private final Vector2d HOME_CALIBRATION = new Vector2d(0, 0);

    // get an instance of each of the subsystems
    AutoConfig autoConfig   = new AutoConfig(this);

    // Declare OpMode members.cvzx
    private DriveSubsystem     driveSubsystem     = new DriveSubsystem(this);
    private SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem(this);
    private TurretSubsystem    turretSubsystem    = new TurretSubsystem(this);
    private PrismSubsystem     prismSubsystem     = new PrismSubsystem(this);
    private LoggingSubsystem   loggingSubsystem   = new LoggingSubsystem(this);

    private ElapsedTime cycleTimer = new ElapsedTime();

    @Override public void runOpMode()
    {
        Globals.IS_AUTO = false;
        Globals.OCTO_ERRORS = 0;
        telemetry.setMsTransmissionInterval(50); //  << make this 100 unless debugging

        autoConfig.initialize();

        // Set GLOBAL flags based on menu choices.
        if (autoConfig.autoOptions.redAlliance ){
            Globals.ALLIANCE_COLOR = AllianceColor.RED;
        } else {
            Globals.ALLIANCE_COLOR = AllianceColor.BLUE;
        }

        Globals.DO_MOTIF = autoConfig.autoOptions.doMotif;

        // Initialize the drive hardware & Turn on telemetry
        driveSubsystem.init(null,true);
        spindexerSubsystem.init(true);
        turretSubsystem.init(true);
        prismSubsystem.init(true);
        //  loggingSubsystem.init(true);  // enable this line to do datalogging.

        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData("ROBOT", "%s - %s\n", Globals.ROBOT_STATE, Globals.ALLIANCE_COLOR);

            if (gamepad1.dpad_up) {
                spindexerSubsystem.cameraUp();
            } else if (gamepad1.dpad_down) {
                spindexerSubsystem.cameraDown();
            }

            driveSubsystem.updatePoseEstimate();
            prismSubsystem.update();
            showCycleTime();
            loggingSubsystem.update();
            telemetry.update();
        }

        spindexerSubsystem.cameraDown();
        spindexerSubsystem.startIntaking();

        while (opModeIsActive())
        {
            telemetry.addData("ROBOT", "%s - %s\n", Globals.ROBOT_STATE, Globals.ALLIANCE_COLOR);

            // Check for a turret home request (because it's drifted.)
            if (gamepad1.rightStickButtonWasPressed() && (turretSubsystem.currentState == TurretStates.READY)) {
                turretSubsystem.setState(TurretStates.INIT);
            }

            // Check to see if we need to home the subsystems
            // Location reset based on Base square and direction of front of robot
            if (gamepad1.touchpad) {
               homeRobot(HOME_CALIBRATION,  0);
            }

            // update the robot's position based on the odometry pods.
            driveSubsystem.updatePoseEstimate();
            spindexerSubsystem.update();
            turretSubsystem.update();
            prismSubsystem.update();

            // use the smart manual drive feature of the DriveSubsystem
            driveSubsystem.smartDrive();
            showCycleTime();

            loggingSubsystem.update();
            telemetry.update();
        }

        // tell AUTO or TELEOP to home next time they run
        Globals.TURRET_HAS_HOMED = false;
        prismSubsystem.setLEDMode(LEDMode.POWER_UP);
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
        double cycleMS = cycleTimer.milliseconds();
        cycleTimer.reset();
         telemetry.addData("Cycle Time", "%.1f mS", cycleMS);
        loggingSubsystem.updateCycle(cycleMS);
    }
}
