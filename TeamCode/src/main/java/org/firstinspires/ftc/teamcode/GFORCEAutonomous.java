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
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStates;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerStates;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name="GFORCE Autonomous", group = "AA" ,  preselectTeleOp="GFORCE Teleop")
public class GFORCEAutonomous extends LinearOpMode
{
    DriveSubsystem  driveSubsystem          = new DriveSubsystem( this);
    SpindexerSubsystem spindexerSubsystem   = new SpindexerSubsystem(this);
    TurretSubsystem turretSubsystem         = new TurretSubsystem(this);
    AutoConfig      autoConfig              = new AutoConfig(this);

    private int    autoMode                 = 0;
    private Action selectedAuto             = null;

    // Configure the starting location for each Auto Mode
    Pose2d atOrigin = new Pose2d(0,0,0);
    Pose2d atGoal   = new Pose2d(-58, -45, Math.toRadians(52));
    Pose2d atWall   = new Pose2d( 62, -16, Math.toRadians(180));
    private final Pose2d[] autoStartLocations = {atOrigin, atGoal, atGoal, atGoal, atGoal, atGoal, atWall, atWall, atWall, atWall};

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
        spindexerSubsystem.sendToShooter(1);
        turretSubsystem.init(true);

            // Wait for driver to press start
        while(opModeInInit()) {

            autoConfig.runMenuUI(); // Run menu system
            // Set GLOBAL flags based on menu choices.
            if (autoConfig.autoOptions.redAlliance )
                Globals.ALLIANCE_COLOR = AllianceColor.RED;
            else
                Globals.ALLIANCE_COLOR = AllianceColor.BLUE;

            // updated needed subsystem
            driveSubsystem.updatePoseEstimate(); // I don't think this is needed since we aren't using encoders anywhere. TEST
            turretSubsystem.update();
            spindexerSubsystem.update();

            telemetry.addLine("\n Touch Play to run Auto");
            telemetry.update();
        }

        Globals.OCTO_ERRORS = 0;
        autoMode = autoConfig.autoOptions.autoMode;

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            selectedAuto = loadSelectedAuto();

            // Do a count down if these is a delayed start,
            for (int sec = autoConfig.autoOptions.delayStart; sec > 0; sec--) {
                telemetry.addData("AUTO MODE",  "%s", autoConfig.autoArray[autoConfig.autoOptions.autoMode]);
                telemetry.addData("COUNTDOWN",  "%d  %d  %d  %d", sec, sec, sec, sec);
                telemetry.update();
                sleep(1000);
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
    }


    // ==========================================================================================
    // Place all auto builders here!
    // ==========================================================================================

    private Action build_Goal9ReleaseAuto() {
        Action backFirstScore = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
            .splineTo(mirror(-24, -14), mirror(0))
            .build();

        Action backCollectPath1 = driveSubsystem.actionBuilder(mirror(-24, -14, 0))
            .splineTo(mirror(-12, -30), mirror(-90))
            .lineToY(mirrorY(-56), new TranslationalVelConstraint(10))
            .build();

        Action releaseReturnPath1 = driveSubsystem.actionBuilder(mirror(-12, -56, -90))
            .setReversed(true)
            .splineTo(mirror(-6, -36), mirror(90))
            .setReversed(false)
            .splineTo(mirror(-4, -56), mirror(-90))
            .waitSeconds(1)
            .build();

        Action backReturnPath1 = driveSubsystem.actionBuilder(mirror(-4, -56, -90))
            .setReversed(true)
            .splineTo(mirror(-24, -14), mirror(-180))
            .build();

        Action backCollectPath2 = driveSubsystem.actionBuilder(mirror(-24, -14, 0))
            .setReversed(false)
            .splineTo(mirror(12, -30), mirror(-90))
            .lineToY(mirrorY(-62), new TranslationalVelConstraint(10))
            .build();

        Action backReturnPath2 = driveSubsystem.actionBuilder(mirror(12, -62, -90))
            .setReversed(true)
            .splineTo(mirror(-36, -14), mirror(-180))
            .build();

        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backFirstScore,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            backCollectPath1,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            releaseReturnPath1,
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnPath1,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            backCollectPath2,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnPath2,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING)
        );
    }

    private Action build_TestAuto() {
        Action drivePath = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
                .lineToX(24)
                .build();

        return new SequentialAction(
                drivePath
        );
    }

    //===========================================================================================
    private Action build_GoalLeave() {
        Action backLeaveGoalPath = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
            .splineTo(mirror(-48, -24), mirror(45))
            .build();

        return new SequentialAction(
            // Score Specimen 1 then sweep 3 more, score 4
            backLeaveGoalPath
        );
    }

    // ==========================================================================================
    private Action build_GoalShoot6(){
        Action backFirstScore = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
            .splineTo(mirror(-24, -14), mirror(0))
            .build();

        Action backCollectPath1 = driveSubsystem.actionBuilder(mirror(-24, -14, 0))
            .splineTo(mirror(-12, -30), mirror(-90))
            .lineToY(mirrorY(-56), new TranslationalVelConstraint(10))
            .build();

        Action backReturnPath1 = driveSubsystem.actionBuilder(mirror(-12, -56, -90))
            .setReversed(true)
            .splineTo(mirror(-36, -14), mirror(-180))
            .build();

        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backFirstScore,
            spindexerSubsystem.actionStartAutoShooting(),

            backCollectPath1,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnPath1,
            spindexerSubsystem.actionStartAutoShooting()
        );
    }

    private Action build_GoalShoot6Release(){
        Action backFirstScore = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
            .splineTo(mirror(-24, -14), mirror(0))
            .build();

        Action backCollectPath1 = driveSubsystem.actionBuilder(mirror(-24, -14, 0))
            .splineTo(mirror(-12, -30), mirror(-90))
            .lineToY(mirrorY(-56), new TranslationalVelConstraint(10))
            .build();

        Action releaseReturnPath1 = driveSubsystem.actionBuilder(mirror(-12, -56, -90))
            .setReversed(true)
            .splineTo(mirror(0, -36), mirror(90))
            .setReversed(false)
            .splineTo(mirror(-1, -56), mirror(-90))
            .waitSeconds(1)
            .build();

        Action backReturnPath1 = driveSubsystem.actionBuilder(mirror(-12, -56, -90))
            .setReversed(true)
            .splineTo(mirror(-36, -14), mirror(-180))
            .build();

        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backFirstScore,
            spindexerSubsystem.actionStartAutoShooting(),

            backCollectPath1,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            releaseReturnPath1,
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnPath1,
            spindexerSubsystem.actionStartAutoShooting()
        );
    }

    //===========================================================================================
    private Action build_GoalShootAndCollect(){

        Action backFirstScore = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
            .splineTo(mirror(-24, -14), mirror(0))
            .build();

        Action backCollectPath1 = driveSubsystem.actionBuilder(mirror(-24, -14, 0))
            .splineTo(mirror(-12, -30), mirror(-90))
            .lineToY(mirrorY(-56), new TranslationalVelConstraint(10))
            .build();

        Action backReturnPath1 = driveSubsystem.actionBuilder(mirror(-12, -56, -90))
            .setReversed(true)
            .splineTo(mirror(-24, -14), mirror(-180))
            .build();

        Action backCollectPath2 = driveSubsystem.actionBuilder(mirror(-24, -14, 0))
            .setReversed(false)
            .splineTo(mirror(12, -30), mirror(-90))
            .lineToY(mirrorY(-62), new TranslationalVelConstraint(10))
            .build();

        Action backReturnPath2 = driveSubsystem.actionBuilder(mirror(12, -62, -90))
            .setReversed(true)
            .splineTo(mirror(-36, -14), mirror(-180))
            .build();

        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backFirstScore,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            backCollectPath1,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnPath1,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            backCollectPath2,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            backReturnPath2,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING)
        );
    }

    //===========================================================================================
    private Action build_FrontLeave() {
        Action frontLeavePath = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
            .lineToX(36)
            .build();

        return new SequentialAction(
            frontLeavePath
        );
    }

    //===========================================================================================
    private Action build_FrontShoot(){

        Action frontFirstScore = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
            .lineToX(54)
            .build();

        Action frontLeavePath = driveSubsystem.actionBuilder(mirror(54, -16, 180))
            .lineToX(36)
            .build();

        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontFirstScore,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),
            spindexerSubsystem.actionStopIntake(),
            frontLeavePath
        );
    }

    //===========================================================================================
    private Action build_FrontShootAndCollect(){

        Action frontFirstScore = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
            .lineToX(54)
            .build();

        Action frontCollectPath1 = driveSubsystem.actionBuilder(mirror( 54, -16, 180))
            .splineTo(mirror(36, -30), mirror(-90))
            .lineToY(mirrorY(-62), new TranslationalVelConstraint(10))
            .build();

        Action frontReturnPath1 = driveSubsystem.actionBuilder(mirror(36, -62, -90))
            .setReversed(true)
            .splineTo(mirror(54, -16), mirror(90))
            .build();

        Action frontCollectPath2 = driveSubsystem.actionBuilder(mirror(54, -16, -90))
            .setReversed(false)
            .lineToY(mirrorY(-54))
            .splineTo(mirror(58, -62), mirror(-60), new TranslationalVelConstraint(10) )
            .build();

        Action frontReturnPath2 = driveSubsystem.actionBuilder(mirror(58, -62, -60))
            .setReversed(true)
            .splineTo(mirror(54, -16), mirror(90))
            .build();

        Action frontCollectPath3 = driveSubsystem.actionBuilder(mirror(54, -16, -90))
            .setReversed(false)
            .splineTo(mirror(48, -50), mirror(-90))
            .lineToY(mirrorY(-62), new TranslationalVelConstraint(5))
            .build();

        Action frontReturnPath3 = driveSubsystem.actionBuilder(mirror(48, -62, -90))
            .setReversed(true)
            .splineTo(mirror(54, -16), mirror(90))
            .build();

        return new SequentialAction(
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontFirstScore,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            frontCollectPath1,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontReturnPath1,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            frontCollectPath2,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontReturnPath2,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            frontCollectPath3,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            frontReturnPath3,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING)
        );
    }
    //===========================================================================================

    Action build_cycleShootAndCollect() {

        Action cycleFirstScore = driveSubsystem.actionBuilder(mirror(autoStartLocations[autoMode]))
            .lineToX(54)
            .build();

        Action cycleTurnAndCollect = driveSubsystem.actionBuilder(mirror(54, -16, 180))
            .turnTo(mirror(-90))
            .lineToY(mirrorY(-62))
            .build();

        Action cycleShootPath1 = driveSubsystem.actionBuilder(mirror(54, -62, -90))
            .setReversed(true)
            .lineToY(mirrorY(-16))
            .build();

        Action cycleCollect1 = driveSubsystem.actionBuilder(mirror(54, -16, -90))
            .setReversed(false)
            .lineToY(mirrorY(-62))
            .build();

        Action cycleCollect2 = driveSubsystem.actionBuilder(mirror(54, -16, -90))
            .setReversed(false)
            .lineToY(mirrorY(-62))
            .waitSeconds(2)
            .build();

        Action cycleShootPath2 = driveSubsystem.actionBuilder(mirror(54, -62, -90))
            .setReversed(true)
            .lineToY(mirrorY(-16))
            .build();


        return new SequentialAction (
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            cycleFirstScore,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            cycleTurnAndCollect,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            cycleShootPath1,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            cycleCollect1,
            spindexerSubsystem.actionWaitForDoneCollecting(1),
            Globals.actionSetRobotState(RobotStates.SHOOTING),
            cycleShootPath2,
            spindexerSubsystem.actionStartAutoShooting(),
            spindexerSubsystem.actionWaitForState(SpindexerStates.INTAKING),

            cycleCollect2,
            spindexerSubsystem.actionWaitForDoneCollecting(1)
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

    private static double mirrorY(double lineToY){
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED){
            lineToY = -lineToY;
        }
        return lineToY;
    }

    private Vector2d mirror(Vector2d positionXY){
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED){
            positionXY = new Vector2d(positionXY.x, -positionXY.y);
        }
        return positionXY;
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
                sequentialAction = build_TestAuto();
                break;

            case 1:
                sequentialAction = build_GoalLeave();
                break;

            case 2:
                sequentialAction = build_GoalShoot6();
                break;

            case 3:
                sequentialAction = build_GoalShoot6Release();
                break;

            case 4:
                sequentialAction = build_GoalShootAndCollect();
                break;

            case 5:
                sequentialAction = build_Goal9ReleaseAuto();
                break;

            case 6:
                sequentialAction = build_FrontLeave();
                break;

            case 7:
                sequentialAction = build_FrontShoot();
                break;

            case 8:
                sequentialAction = build_FrontShootAndCollect();
                break;

            case 9:
                sequentialAction = build_cycleShootAndCollect();
                break;

            default:
                sequentialAction = new SequentialAction();

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
