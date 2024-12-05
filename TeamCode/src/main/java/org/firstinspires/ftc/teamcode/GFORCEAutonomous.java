/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.HOME;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.LOWERING;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.SAMPLE_HELD;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmStates;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Autonomous(name="GFORCE Autonomous", group = "AA" ,  preselectTeleOp="GFORCE Teleop")
public class GFORCEAutonomous extends LinearOpMode
{
    MecanumDrive robot;
    AutoConfig autoConfig   = new AutoConfig(this);
    ArmSubsystem arm        = new ArmSubsystem(this);
    IntakeSubsystem intake  = new IntakeSubsystem(this);
    LiftSubsystem lift      = new LiftSubsystem(this);
//  OctoQuadIF octoQuad     = new OctoQuadIF(this);
//  VisionSubsystem blob    = new VisionSubsystem(this);

    private Action selectedAuto  = null;
    private int lastSelectedAuto = -1;

    // Place all auto builders here!

    //================================================================================================================
    private Action buildSpecimen_4Sub() {
        robot = new MecanumDrive(hardwareMap, new Pose2d(4, -63, Math.toRadians(90)));

        //build trajectories
        Action wallToSubPath = robot.actionBuilder(new Pose2d(4, -63, Math.toRadians(90)))
                .lineToY(-31)
                .build()
                ;

        Action subToAllSamplesPath = robot.actionBuilder(new Pose2d(4, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-60))
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(44, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(44, -50), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(54, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(54, -50), Math.toRadians(-90))
                .build()
                ;

        Action samplesToSpecimenPath = robot.actionBuilder(new Pose2d(54, -50, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(30, -50), Math.toRadians(180))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(30, -63), Math.toRadians(-90))
                .build() ;

        Action specimenToSub2Path = robot.actionBuilder(new Pose2d(30, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(145))
                .splineToConstantHeading(new Vector2d(0, -31), Math.toRadians(90))
                .build();

        Action sub2ToSpecimenPath = robot.actionBuilder(new Pose2d(0, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-35))
                .splineToConstantHeading(new Vector2d(30, -63), Math.toRadians(-90))
                .build();

        Action specimenToSub3Path = robot.actionBuilder(new Pose2d(30, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(new Vector2d(-4, -31), Math.toRadians(90))
                .build();

        Action sub3ToSpecimenPath = robot.actionBuilder(new Pose2d(-4, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-30))
                .splineToConstantHeading(new Vector2d(30, -63), Math.toRadians(-90))
                .build();

        Action specimenToSub4Path = robot.actionBuilder(new Pose2d(30, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(155))
                .splineToConstantHeading(new Vector2d(-8, -31), Math.toRadians(90))
                .build();

        Action sub4ToObservationPath = robot.actionBuilder(new Pose2d(-8, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-30))
                .splineToConstantHeading(new Vector2d(50, -56), Math.toRadians(0))
                .build();

        //  ######################################################################

        Action autoSequence = new SequentialAction(
                // Score Specimen 1 then sweep 2 more
                arm.actionSetState(ArmStates.GRABBED),
                wallToSubPath,
                arm.actionClipIt(),
                arm.actionWaitForState(ArmStates.LOWERING),
                subToAllSamplesPath,
                samplesToSpecimenPath,
                // Pickup and score Specimen 2
                arm.actionSetState(ArmStates.GRABBING),
                arm.actionWaitForState(ArmStates.GRABBED),
                specimenToSub2Path,
                arm.actionClipIt(),
                arm.actionWaitForState(ArmStates.LOWERING),
                sub2ToSpecimenPath,
                // Pickup and score Specimen 3
                arm.actionSetState(ArmStates.GRABBING),
                arm.actionWaitForState(ArmStates.GRABBED),
                specimenToSub3Path,
                arm.actionClipIt(),
                arm.actionWaitForState(ArmStates.LOWERING),
                sub3ToSpecimenPath,
                // Pickup and score Specimen 4
                arm.actionSetState(ArmStates.GRABBING),
                arm.actionWaitForState(ArmStates.GRABBED),
                specimenToSub4Path,
                arm.actionClipIt(),
                arm.actionWaitForState(ArmStates.LOWERING),
                // Go Park
                sub4ToObservationPath
        );

        return  new ParallelAction(
                arm.actionUpdate(),
                autoSequence
        );
    }

    //================================================================================================================
    private Action buildSample_1Bas() {
        robot = new MecanumDrive(hardwareMap, new Pose2d(-32, -63, Math.toRadians(90)));

        Action wallToBasket = robot.actionBuilder(new Pose2d(-32, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-54, -58, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build()
                ;

        Action basketToSamples = robot.actionBuilder(new Pose2d(-54, -58, Math.toRadians(45)))
                .turnTo(Math.toRadians(90))
                .build();

        Action autoSequence = new SequentialAction(
                wallToBasket ,
                lift.actionSetState(SAMPLE_HELD),
                lift.actionWaitForState(LOWERING),
                basketToSamples
        );

        return  new ParallelAction(
                lift.actionUpdate(),
                autoSequence
        );
    }

    //================================================================================================================
    private Action buildSample_1Bas_3Net() {
        robot = new MecanumDrive(hardwareMap, new Pose2d(-32, -63, Math.toRadians(90)));

        Action wallToBasket = robot.actionBuilder(new Pose2d(-32, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-54, -58, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build()
                ;

        Action basketToSamples = robot.actionBuilder(new Pose2d(-54, -58, Math.toRadians(45)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-36, -48, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-36, -24, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-44, -24, Math.toRadians(90)), Math.toRadians(-90))
                /*
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(-135))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-44, -24, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-49, -12, Math.toRadians(90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-53, -24, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-53, -56, Math.toRadians(90)), Math.toRadians(-90))
                */

                .build();

        Action autoSequence = new SequentialAction(
                wallToBasket ,
                lift.actionSetState(SAMPLE_HELD),
                lift.actionWaitForState(LOWERING),
                basketToSamples
        );

        return  new ParallelAction(
                lift.actionUpdate(),
                autoSequence
        );
    }

    @Override
    public void runOpMode()
    {
        Globals.IS_AUTO = true;

        arm.initialize(false);
        arm.closeClaw();
        lift.initialize(false);
        intake.initialize(false);
        autoConfig.initialize();
//      octoQuad.initialize(false);
//      blob.initilaize(false);

        // ############################################################################


        // Wait for driver to press start
        while(opModeInInit()) {

            autoConfig.runMenuUI(); //Run menu system

            // Note:  The cases below MUST match the order of auto options in the AutoConfi.java file.
            if (autoConfig.autoOptions.autoMode != lastSelectedAuto) {
                lastSelectedAuto = autoConfig.autoOptions.autoMode;
                switch (lastSelectedAuto) {
                    case 0:
                        selectedAuto = buildSpecimen_4Sub();
                        break;

                    case 1:
                        selectedAuto = buildSample_1Bas();
                        break;

                    case 2:
                        selectedAuto = buildSample_1Bas_3Net();
                        break;
                }
            }

            /* Set GLOBAL flags based on menu choices.
            if (autoConfig.autoOptions.redAlliance )
                Globals.ALLIANCE_COLOR = AllianceColor.RED;
            else
                Globals.ALLIANCE_COLOR = AllianceColor.BLUE;
            */

            telemetry.addLine("\n Touch Play to run Auto");
            telemetry.update();
        }

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
                telemetry.addData("AUTO MODE",  "No valid mode selected (%d)", autoConfig.autoOptions.autoMode);
                telemetry.update();
                sleep(5000);
            }

        }
    }

    /*
    final double WITHIN_RANGE   = 2.00 ;
    final double APPROACH_SPEED = -0.3 ;
    final double CLICK_ON_SPEED = -0.2 ;
    final double STRAFE_GAIN    = 0.7 ;

    public Action actionDriveToSpecimen(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){

                boolean keepGoing = true;
                octoQuad.update();
                double yError = octoQuad.getBackRangeInches();
                robot.updatePoseEstimate();

                // try to get within range of specimen
                if ((Math.abs(yError) > WITHIN_RANGE) || (yError == 0)) {
                    double strafe = 0;
                    double drive = 0;
                    double xError = 0;

                    // strafe to be in front of specimen
                    ColorTarget target = blob.getTarget();
                    if (target.valid) {
                        xError = target.x;
                        strafe = xError * STRAFE_GAIN;
                    }

                    // slow down when you get close to wall.
                    if (yError >= 5) {
                        drive = APPROACH_SPEED;
                    } else {
                        drive = CLICK_ON_SPEED;
                    }

                    // send drive power to wheels, and continue action
                    robot.setDrivePowers(new PoseVelocity2d(new Vector2d(drive, strafe), 0));
                } else {
                    // stop moving and exit action.
                    robot.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
                    keepGoing = false;
                }

                telemetry.update();
                return keepGoing;
            }
        };
    }
    */
}
