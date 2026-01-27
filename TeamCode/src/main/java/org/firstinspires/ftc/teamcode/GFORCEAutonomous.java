/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.LEDMode;
import org.firstinspires.ftc.teamcode.subsystems.LoggingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PrismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStates;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerStates;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name="GFORCE Autonomous", group = "AA" ,  preselectTeleOp="GFORCE Teleop")
public class GFORCEAutonomous extends LinearOpMode
{
    private DriveSubsystem  driveSubsystem          = new DriveSubsystem( this);
    private SpindexerSubsystem spindexerSubsystem   = new SpindexerSubsystem(this);
    private TurretSubsystem turretSubsystem         = new TurretSubsystem(this);
    private PrismSubsystem  prismSubsystem          = new PrismSubsystem(this);
    private AutoConfig      autoConfig              = new AutoConfig(this);
    private LoggingSubsystem loggingSubsystem   = new LoggingSubsystem(this);


    private int    autoMode                 = 0;
    private Action selectedAuto             = null;

    // Configure the starting location for each Auto Mode
    Pose2d atOrigin = new Pose2d(0,0,0);
    Pose2d atGoal   = new Pose2d(-58, -45, Math.toRadians(52));
    Pose2d atWall   = new Pose2d( 62, -15, Math.toRadians(180));
    private final Pose2d[] autoStartLocations = {atGoal, atGoal, atGoal, atGoal, atGoal, atGoal, atWall, atWall, atWall, atOrigin};
    private AllianceColor lastAllianceColor = AllianceColor.UNKNOWN;
    private int lastAutoMode = -1;

    // ############################################################################

    @Override
    public void runOpMode()
    {
        Globals.IS_AUTO = true;
        telemetry.setMsTransmissionInterval(50);
        autoConfig.initialize();

        driveSubsystem.init(null, true);
        spindexerSubsystem.init(true);
        spindexerSubsystem.preloadSequence();
        spindexerSubsystem.sendToShooter(0);   /// change to 0 for no-move auto
        turretSubsystem.init(true);
        prismSubsystem.init(true);
        loggingSubsystem.init(true);  // enable this line to do datalogging.


        // Wait for driver to press start
        while(opModeInInit()) {

            autoConfig.runMenuUI(); // Run menu system
            // Set GLOBAL flags based on menu choices.
            if (autoConfig.autoOptions.redAlliance )
                Globals.ALLIANCE_COLOR = AllianceColor.RED;
            else
                Globals.ALLIANCE_COLOR = AllianceColor.BLUE;

            Globals.DO_MOTIF = autoConfig.autoOptions.doMotif;
            if (Globals.DO_MOTIF){
                spindexerSubsystem.enableVision();  // this only does anything if it's not enabled
            }

            // Set the Auto Mode and load the path sequence and Starting Location.
            autoMode = autoConfig.autoOptions.autoMode;
            if ((lastAutoMode != autoMode) || (lastAllianceColor != Globals.ALLIANCE_COLOR)) {
                selectedAuto = loadSelectedAuto();
                driveSubsystem.updatePoseEstimate();

                lastAllianceColor = Globals.ALLIANCE_COLOR;
                lastAutoMode = autoMode;
            }

            // updated needed subsystem

            turretSubsystem.update();
            spindexerSubsystem.update();
            prismSubsystem.update();

            telemetry.addLine("\n Touch Play to run Auto");
            telemetry.update();
        }

        Globals.OCTO_ERRORS = 0;


        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // selectedAuto = loadSelectedAuto();

            // Do a count down if there is a delayed start,
            for (int sec = autoConfig.autoOptions.delayStart; sec > 0; sec--) {
                telemetry.addData("AUTO MODE",  "%s", autoConfig.autoArray[autoConfig.autoOptions.autoMode]);
                telemetry.addData("COUNTDOWN",  "%d  %d  %d  %d", sec, sec, sec, sec);
                telemetry.update();
                sleep(1000);
            }

            if (Globals.DO_MOTIF) {
                spindexerSubsystem.cameraUp();
            }

            if (selectedAuto != null) {
                Actions.runBlocking(selectedAuto);
                spindexerSubsystem.stopIntake();
            } else {
                telemetry.addData("AUTO MODE",  "No valid mode selected");
                telemetry.update();
            }
        }

        spindexerSubsystem.stopIntake();
        Globals.LAST_POSE = driveSubsystem.getPose() ;
        prismSubsystem.setLEDMode(LEDMode.POWER_UP);
    }

    // ==========================================================================================
    // Place all Back-Side paths here!
    // ==========================================================================================

    // create methods for each individual path
    private Action goalScorePreloads() {
        return driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
            .lineToY(mirrorY(-20))
            .build();
    }

    private Action goalSmartMotifTurn(){
        if (Globals.DO_MOTIF){
            return new SequentialAction(
                turnToZero(),
                spindexerSubsystem.actionReadMotif(),
                spindexerSubsystem.actionStartAutoShooting()
            );
        } else {
            return new SequentialAction(
                spindexerSubsystem.actionStartAutoShooting(),
                turnToZero()
            );
        }
    }

    private Action turnToZero() {
        return driveSubsystem.actionBuilder(mirror(-39, -20, 52))
            .turnTo(mirror(0))
            .build();
    }

    private Action backCollectRow1() {
        return driveSubsystem.actionBuilder(mirror(-39, -20, 0))
            .splineTo(mirror(-12, -42), mirror(-90), new TranslationalVelConstraint(20))
            .lineToY(mirrorY(-58), new TranslationalVelConstraint(12))
            .build();
    }

    private Action backReleaseAfterRow1() {
        return driveSubsystem.actionBuilder(mirror(-12, -56, -90))
            .setReversed(true)
            .splineTo(mirror(-6, -36), mirror(90))
            .setReversed(false)
            .splineTo(mirror(-4, -56), mirror(-90))
            .waitSeconds(1)
            .build();
    }

    private Action backReturnAfterRelease() {
        return driveSubsystem.actionBuilder(mirror(-4, -56, -90))
            .setReversed(true)
            .splineTo(mirror(-12, -20), mirror(90))
            .build();
    }

    private Action backReturnAfterReleaseFinal() {
        return driveSubsystem.actionBuilder(mirror(-4, -56, -90))
            .setReversed(true)
            .splineTo(mirror(-40, -20), mirror(180))
            .build();
    }

    private Action backReturnRow1Final() {
        return driveSubsystem.actionBuilder(mirror(-12, -56, -90))
            .setReversed(true)
            .splineTo(mirror(-40, -20), mirror(150))
            .build();
    }

    private Action backReturnRow1() {
        return driveSubsystem.actionBuilder(mirror(-12, -56, -90))
            .setReversed(true)
            .lineToY(mirrorY(-20))
            .build();
    }

    private Action backCollectRow2() {
        return driveSubsystem.actionBuilder(mirror(-12, -20, -90))
            .setReversed(false)
            .turnTo(mirror(0))
            .splineTo(mirror(15, -42), mirror(-90), new TranslationalVelConstraint(20))
            .lineToY(mirrorY(-64), new TranslationalVelConstraint(12))
            .build();
    }

    private Action backReturnRow2Final() {
        return driveSubsystem.actionBuilder(mirror(15, -56, -90))
            .setReversed(true)
            .lineToY(mirrorY(-48))
            .splineTo(mirror(-12, -20), mirror(180))
            .lineToX(-40)
            .build();
    }

    private Action backReturnRow2() {
        return driveSubsystem.actionBuilder(mirror(15, -56, -90))
            .setReversed(true)
            .lineToY(mirrorY(-44))
            .splineTo(mirror(-12, -20), mirror(180))
            .lineToX(-12)
            .build();
    }

    private Action backCollectRow3() {
        return driveSubsystem.actionBuilder(mirror(-12, -20, 0))
            .setReversed(false)
            .lineToX(12)
            .splineTo(mirror(36, -42), mirror(-90), new TranslationalVelConstraint(20))
            .lineToY(mirrorY(-64), new TranslationalVelConstraint(12))
            .waitSeconds(1)
            .build();
    }

    private Action backReturnRow3Final() {
        return driveSubsystem.actionBuilder(mirror(36, -62, -90))
            .lineToY(mirrorY(-44))
            .splineTo(mirror(12, -20), mirror(180))
            .lineToX(-40)
            .build();
    }

    // ==========================================================================================
    // Place all Front-side paths here!
    // ==========================================================================================

    private Action frontScorePreloads() {
        return driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
                .lineToX(54)
                .build();
    }

    private Action frontLeave() {
        return  driveSubsystem.actionBuilder(mirror(54, -13, 180))
                .lineToX(36)
                .build();
    }

    private Action frontCollectRow3() {
        return driveSubsystem.actionBuilder(mirror(54, -16, 180))
                .splineTo(mirror(36, -30), mirror(-90))
                .lineToY(mirrorY(-64), new TranslationalVelConstraint(12))
                .build();
    }

    private Action frontReturnRow3() {
        return driveSubsystem.actionBuilder(mirror(36, -62, -90))
                .setReversed(true)
                .splineTo(mirror(54, -16), mirror(90))
                .build();
    }

    private Action frontTurnToMinus90() {
        return driveSubsystem.actionBuilder(mirror(54, -16, 180))
                .turnTo(mirror(-90))
                .build();
    }

    private Action frontCollectCycle() {
        return driveSubsystem.actionBuilder(mirror(54, -16, -90))
                .lineToY(mirrorY(-56))
                .lineToY(mirrorY(-64), new TranslationalVelConstraint(12))
                .build();
    }

    private Action frontReturnCycle() {
        return driveSubsystem.actionBuilder(mirror(54, -62, -90))
                .setReversed(true)
                .lineToY(mirrorY(-18))
                .lineToY(mirrorY(-16), new TranslationalVelConstraint(12))
                .waitSeconds(0.33)
                .build();
    }

    private Action frontCollectMoreCycle() {
        return driveSubsystem.actionBuilder(mirror(54, -16, -90))
                .splineTo(mirror(38, -56), mirror(-90))
                .lineToY(mirrorY(-64), new TranslationalVelConstraint(12))
                .build();
    }

    private Action frontReturnMoreCycle() {
        return driveSubsystem.actionBuilder(mirror(38, -62, -90))
                .setReversed(true)
                .splineTo(mirror(54, -18), mirror(90))
                .lineToY(mirrorY(-16), new TranslationalVelConstraint(12))
                .waitSeconds(0.33)
                .build();
    }

    // ==========================================================================================
    // Place all BACK Auto builders here!
    // ==========================================================================================

    private Action build_Goal_3(){
        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            goalScorePreloads(),
            goalSmartMotifTurn(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING)
        );
    }

    // ==========================================================================================
    private Action build_Goal_6(){

        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            goalScorePreloads(),
            goalSmartMotifTurn(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            backCollectRow1(),
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnRow1Final(),
            spindexerSubsystem.actionStartAutoShooting()
        );
    }

    //===========================================================================================
    private Action build_Goal_3_R_3(){
        return new SequentialAction(
                Globals.actionSetRobotState(RobotStates.SHOOTING),
                goalScorePreloads(),
                goalSmartMotifTurn(),
                spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

                backCollectRow1(),
                spindexerSubsystem.actionWaitForDoneCollecting(1),
                backReleaseAfterRow1(),
                Globals.actionSetRobotState(RobotStates.SHOOTING),
                backReturnAfterReleaseFinal(),
                spindexerSubsystem.actionStartAutoShooting()
        );
    }

    //===========================================================================================
    private Action build_Goal_9(){

        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            goalScorePreloads(),
            goalSmartMotifTurn(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            backCollectRow1(),
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnRow1(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            backCollectRow2(),
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnRow2Final(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING)
        );
    }

    //===========================================================================================
    private Action build_Goal_3_R_6() {
        return new SequentialAction(
                Globals.actionSetRobotState(RobotStates.SHOOTING),
                goalScorePreloads(),
                goalSmartMotifTurn(),
                spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

                backCollectRow1(),
                spindexerSubsystem.actionWaitForDoneCollecting(1),
                backReleaseAfterRow1(),
                Globals.actionSetRobotState(RobotStates.SHOOTING),
                backReturnAfterRelease(),
                spindexerSubsystem.actionStartAutoShooting(),
                spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

                backCollectRow2(),
                spindexerSubsystem.actionWaitForDoneCollecting(1),
                Globals.actionSetRobotState(RobotStates.SHOOTING),
                backReturnRow2Final(),
                spindexerSubsystem.actionStartAutoShooting(),
                spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING)
        );
    }

    //===========================================================================================
    private Action build_Goal_12() {
        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            goalScorePreloads(),
            goalSmartMotifTurn(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            backCollectRow1(),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnRow1(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            backCollectRow2(),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnRow2(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            backCollectRow3(),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnRow3Final(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING)
        );
    }


    // ==========================================================================================
    // Place all FRONT Auto builders here!
    // ==========================================================================================

    private Action build_Front_3() {
        return new SequentialAction(
                Globals.actionSetRobotState(RobotStates.SHOOTING),
                frontScorePreloads(),
                spindexerSubsystem.actionStartAutoShooting(),
                spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),
                spindexerSubsystem.actionStopIntake(),
                frontLeave()
        );    }

    //===========================================================================================
    private Action build_Front_6_C_3(){

        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontScorePreloads(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            frontCollectRow3(),
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontReturnRow3(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            frontCollectCycle(),
            spindexerSubsystem.actionWaitForDoneCollecting(1.5),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontReturnCycle(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            frontCollectMoreCycle(),
            spindexerSubsystem.actionWaitForDoneCollecting(1.5),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontReturnMoreCycle(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            frontCollectCycle(),
            spindexerSubsystem.actionWaitForDoneCollecting(2)
        );
    }
    //===========================================================================================

    Action build_Front_3_C_6() {
        return new SequentialAction (
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontScorePreloads(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),
            frontTurnToMinus90(),

            frontCollectCycle(),
            spindexerSubsystem.actionWaitForDoneCollecting(1.5),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontReturnCycle(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            frontCollectCycle(),
            spindexerSubsystem.actionWaitForDoneCollecting(1.5),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontReturnCycle(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            frontCollectCycle(),
            spindexerSubsystem.actionWaitForDoneCollecting(1.5),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontReturnCycle(),
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            frontCollectCycle()
            );
    }

    //===========================================================================================
    //   Build test sequence
    //===========================================================================================
    private Action build_SteerTest() {
        Action test = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
                .lineToX(48)
                .splineTo(mirror(72, 24), mirror(90))
                .lineToY(mirrorY(72))
                .build();

        return new SequentialAction(
                test
        );
    }

    //===========================================================================================

    private double mirror(double headingDeg){
        double headingRad = Math.toRadians(headingDeg);
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED){
            headingRad = -headingRad;
        }
        return headingRad;
    }

    private double mirrorY(double lineToY){
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED){
            lineToY = -lineToY;
        }
        return lineToY;
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

    private Pose2d mirror(double x, double y, double headingDeg){
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED){
            return new Pose2d(x, -y, Math.toRadians(-headingDeg));
        } else {
            return new Pose2d(x, y, Math.toRadians(headingDeg));
        }
    }

    /**
     * Take the current auto mode and build the matching RoadRunner sequence.
     * Save the last auto value for outside comparison.
     * @return
     */
    private Action loadSelectedAuto() {

        // Save the latest chanages for next time arounf INIT loop.
        driveSubsystem.setPose(mirror(autoStartLocations[autoMode]));

        // Note:  The cases below MUST match the order of auto options in the AutoConfig.java file.
        Action sequentialAction;
        switch (autoMode) {
            case 0:
                sequentialAction = build_Goal_3();
                break;

            case 1:
                sequentialAction = build_Goal_6();
                break;

            case 2:
                sequentialAction = build_Goal_3_R_3();
                break;

            case 3:
                sequentialAction = build_Goal_9();
                break;

            case 4:
                sequentialAction = build_Goal_3_R_6();
                break;

            case 5:
                sequentialAction = build_Goal_12();
                break;

            case 6:
                sequentialAction = build_Front_3();
                break;

            case 7:
                sequentialAction = build_Front_6_C_3();
                break;

            case 8:
                sequentialAction = build_Front_3_C_6();
                break;

            case 9:
                sequentialAction = build_SteerTest();
                break;

            default:
                sequentialAction = new SequentialAction();
                break;
        }

        // Run 4 actions simultaniously
        return  new ParallelAction(
                turretSubsystem.actionUpdate(),
                spindexerSubsystem.actionUpdate(),
                prismSubsystem.actionUpdate(),
                loggingSubsystem.actionUpdate(),

                sequentialAction,
                turretSubsystem.actionTelemetryUpdate()  // just update telemetry
        );
    }
}
