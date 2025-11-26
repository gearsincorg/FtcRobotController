/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStates;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerStates;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.opencv.core.Mat;

@Autonomous(name="GFORCE Autonomous", group = "AA" ,  preselectTeleOp="GFORCE Teleop")
public class GFORCEAutonomous extends LinearOpMode
{
    DriveSubsystem driveSubsystem = new DriveSubsystem( this);
    SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem(this);
    TurretSubsystem turretSubsystem    = new TurretSubsystem(this);
    AutoConfig autoConfig   = new AutoConfig(this);

    private Action selectedAuto  = null;
    private int lastSelectedAuto = -1;

    private final double START_X = 62;
    private final double START_Y = 16;
    private final double START_H = Math.toRadians(180);
    private final int NUMBERS_OF_AUTOS = 1;
    private final Pose2d[] startLocations = new Pose2d[NUMBERS_OF_AUTOS];


    // ==========================================================================================
    // Place all auto builders here!
    // ==========================================================================================
    private Action build_LeaveGoal() {
        driveSubsystem.setPose(mirror(startLocations[0]));

        Action drivePath = driveSubsystem.actionBuilder(mirror(startLocations[0]))
                .splineTo(mirror(-48, -24), mirror(45))
                .build();

        return new SequentialAction(
                // Score Specimen 1 then sweep 3 more, score 4
                drivePath
        );
    }

    //===========================================================================================

    private Action build_TestAuto() {
        driveSubsystem.setPose(mirror(startLocations[0]));

        Action drivePath = driveSubsystem.actionBuilder(mirror(startLocations[0]))
                .splineTo(mirror(-48, 24), mirror(45))
                .build();

        return new SequentialAction(
                drivePath
        );
    }

    // ==========================================================================================
    private Action build_GoalShoot(){
        driveSubsystem.setPose(mirror(startLocations[0]));

        Action drivePath = driveSubsystem.actionBuilder(mirror(startLocations[0]))
                .splineTo(mirror(-48, -24), mirror(45))
                .build();

        return new SequentialAction(
            turretSubsystem.actionSetupShooter(20, 9, 0),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            drivePath,
            spindexerSubsystem.actionStartAutoShooting()
        );
    }

    private Action build_GoalShootAndCollect(){
        driveSubsystem.setPose(mirror(startLocations[0]));

        Action drivePath = driveSubsystem.actionBuilder(mirror(startLocations[0]))
                .splineTo(mirror(-48, -24), mirror(45))
                .build();

        Action collectPath = driveSubsystem.actionBuilder(mirror(new Pose2d(-48, -24, mirror(45))))
                .splineTo(mirror(-12, -12), mirror(0))
                .turnTo(mirror(90))
                .splineTo(mirror(-12, -24), mirror(90))
                .build();


        return new SequentialAction(
                turretSubsystem.actionSetupShooter(20, 9, 0),
                Globals.actionSetRobotState(RobotStates.SHOOTING),
                drivePath,
                spindexerSubsystem.actionStartAutoShooting(),
                spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),
                collectPath
        );
    }

    // ############################################################################

    @Override
    public void runOpMode()
    {
        Globals.IS_AUTO = true;
        autoConfig.initialize();
        startLocations[0] = new Pose2d(-58, -45, Math.toRadians(52));
        driveSubsystem.init(new Pose2d(0,0,0), true);
        spindexerSubsystem.init(true);
        spindexerSubsystem.preloadSequence();
        turretSubsystem.init(true);
        selectedAuto = loadSelectedAuto(autoConfig.autoOptions.autoMode);  // build the current auto sequence

        // Wait for driver to press start
        while(opModeInInit()) {

            autoConfig.runMenuUI(); // Run menu system

            // Set GLOBAL flags based on menu choices.
            if (autoConfig.autoOptions.redAlliance )
                Globals.ALLIANCE_COLOR = AllianceColor.RED;
            else
                Globals.ALLIANCE_COLOR = AllianceColor.BLUE;

            // If auto mode changes, load the new mode.
            if (autoConfig.autoOptions.autoMode != lastSelectedAuto) {
                lastSelectedAuto = autoConfig.autoOptions.autoMode;
                selectedAuto = loadSelectedAuto(autoConfig.autoOptions.autoMode);
            }

            // updated needed subsystem
            driveSubsystem.updatePoseEstimate();
            turretSubsystem.update();
            spindexerSubsystem.update();

            telemetry.addLine("\n Touch Play to run Auto");
            telemetry.update();
        }

        Globals.OCTO_ERRORS = 0;

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Do a count down if these is a delayed start,
            for (int sec = autoConfig.autoOptions.delayStart; sec > 0; sec--) {
                telemetry.addData("AUTO MODE",  "%s", autoConfig.autoArray[autoConfig.autoOptions.autoMode]);
                telemetry.addData("COUNTDOWN",  "%d  %d  %d  %d", sec, sec, sec, sec);
                telemetry.update();
                sleep(1000);
            }

            if (selectedAuto != null) {
                Actions.runBlocking(selectedAuto);
            } else {
                telemetry.addData("AUTO MODE",  "No valid mode selected");
                telemetry.update();
            }
        }

        Globals.LAST_POSE = driveSubsystem.getPose() ;
    }

    private double mirror(double headingDeg){
        double headingRad = Math.toRadians(headingDeg);
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED){
            headingRad = -headingRad;
        }
        return headingRad;
    }

    private Vector2d mirror(Vector2d position){
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED){
            position = new Vector2d(position.x, -position.y);
        }
        return position;
    }

    private Vector2d mirror(double x, double y){
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED){
            return new Vector2d(x, -y);
        }else{
           return new Vector2d(x, y);
        }
    }

    private Pose2d mirror(Pose2d pose){
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED){
            pose = new Pose2d(pose.position.x, -pose.position.y, -pose.heading.toDouble());
        }
        return pose;
    }

    /**
     * Take the current auto mode and build the matching RadRunner sequence.
     * Save the last auto value for outside comparison.
     * @param autoMode
     * @return
     */
    private Action loadSelectedAuto(int autoMode) {

        Action sequentialAction = new SequentialAction();
        lastSelectedAuto = autoMode;

        // Note:  The cases below MUST match the order of auto options in the AutoConfig.java file.
        switch (autoMode) {
            case 0:
                sequentialAction = build_LeaveGoal();
                break;

            case 1:
                sequentialAction = build_GoalShoot();
                break;

            case 2:
                sequentialAction = build_TestAuto();
                break;

            case 3:
                sequentialAction = build_GoalShootAndCollect();
                break;
        }

        // Run 4 actions simultaniously
        return  new ParallelAction(
                turretSubsystem.actionUpdate(),
                spindexerSubsystem.actionUpdate(),
                sequentialAction,
                turretSubsystem.actionTelemetryUpdate()  // just update telemetry
        );
    }
}
