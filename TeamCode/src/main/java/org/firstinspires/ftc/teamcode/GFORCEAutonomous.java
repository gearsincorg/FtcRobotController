/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.teamcode.subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.ArmStates;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.IntakeStates;
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

    private Action selectedAuto  = null;
    private int lastSelectedAuto = -1;

    private final double START_Y = -62;
    private final double START_X_SPEC = 15;
    private final double START_X_SAMP = -33;

    // Place all auto builders here!
    //================================================================================================================
    private Action build_Spec_4Sub() {
        robot.setPose(new Pose2d(START_X_SPEC, START_Y, Math.toRadians(90)));

        //build trajectories
        Action wallToSubPath = robot.actionBuilder(new Pose2d(START_X_SPEC, START_Y, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(4, -31), Math.toRadians(90))
                .build();

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
                .build();

        Action samplesToSpecimenPath = robot.actionBuilder(new Pose2d(54, -50, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(41, -44), Math.toRadians(180))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(41, -63), Math.toRadians(-90))
                .build();

        Action specimenToSub2Path = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(130))
                .splineToConstantHeading(new Vector2d(1, -31), Math.toRadians(90))
                .build();

        Action sub2ToSpecimenPath = robot.actionBuilder(new Pose2d(1, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-50))
                .splineToConstantHeading(new Vector2d(41, -63), Math.toRadians(-90))
                .build();

        Action specimenToSub3Path = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-2, -31), Math.toRadians(90))
                .build();

        Action sub3ToSpecimenPath = robot.actionBuilder(new Pose2d(-2, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(41, -63), Math.toRadians(-90))
                .build();

        Action specimenToSub4Path = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(140))
                .splineToConstantHeading(new Vector2d(-5, -31), Math.toRadians(90))
                .build();

        Action sub4ToObservationPath = robot.actionBuilder(new Pose2d(-5, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-40))
                .splineToConstantHeading(new Vector2d(50, -56), Math.toRadians(0))
                .build() ;

        //  ######################################################################

        return new SequentialAction(
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
    }

    //================================================================================================================
    private Action build_Samp_1Bas() {
        robot.setPose(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)));

        Action wallToBasket = robot.actionBuilder(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-53, -57, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basketToSamples = robot.actionBuilder(new Pose2d(-53, -57, Math.toRadians(45)))
                .turnTo(Math.toRadians(90))
                .build();

        return new SequentialAction(
                arm.actionClaw(true),
                wallToBasket ,
                lift.actionSetState(SAMPLE_HELD),
                lift.actionWaitForHomeOrState(LOWERING),
                basketToSamples
        );
    }

    //================================================================================================================
    private Action build_Samp_1Bas_Sub() {
        robot.setPose(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)));

        Action wallToBasket = robot.actionBuilder(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-53, -57, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basketToSub = robot.actionBuilder(new Pose2d(-53, -57, Math.toRadians(45)))
                .splineTo(new Vector2d(-43, -47), Math.toRadians(45))
                .splineTo(new Vector2d(-28, -10), Math.toRadians(0))
                .splineTo(new Vector2d(-23, -10), Math.toRadians(0), new TranslationalVelConstraint(5.0))
                .build();

        return new SequentialAction(
                arm.actionClaw(true),
                wallToBasket ,
                lift.actionSetState(SAMPLE_HELD),
                lift.actionWaitForHomeOrState(LOWERING),
                arm.actionSetState(ArmStates.GRABBED),
                basketToSub
                );
    }

    //================================================================================================================
    private Action build_Samp_1Bas_2Net_Sub() {
        robot.setPose(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)));

        Action wallToBasket = robot.actionBuilder(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-53, -57, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basketToSamples = robot.actionBuilder(new Pose2d(-53, -57, Math.toRadians(45)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-36, -24, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-44, -24, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-54, -58, Math.toRadians(45)), Math.toRadians(-135))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-44, -24, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-49, -12, Math.toRadians(90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-53, -24, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-53, -56, Math.toRadians(90)), Math.toRadians(-90))
                .build();

        Action basketToSub = robot.actionBuilder(new Pose2d(-53, -56, Math.toRadians(90)))
                .splineTo(new Vector2d(-36, -10), Math.toRadians(0))
                .splineTo(new Vector2d(-24, -10), Math.toRadians(0), new TranslationalVelConstraint(5.0))
                .build();

        return new SequentialAction(
                arm.actionClaw(true),
                wallToBasket ,
                lift.actionSetState(SAMPLE_HELD),
                lift.actionWaitForHomeOrState(LOWERING),
                basketToSamples,
                arm.actionSetState(ArmStates.GRABBED),
                basketToSub
                );
    }

    //==============================================================================================
    private Action build_Spec_1sub_Sample_1bas() {
        robot.setPose(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)));

        Action wallToSub = robot.actionBuilder(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)))
                .setTangent(Math.toRadians(35))
                .splineToConstantHeading(new Vector2d(-15, -33), Math.toRadians(90))  // was -15, -31
                .build();

        Action subToSample1 = robot.actionBuilder(new Pose2d(-15, -33, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-49, -40), Math.toRadians(90))
                .build();

        Action sample1ToBasket = robot.actionBuilder(new Pose2d(-49, -40, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basToSample2 = robot.actionBuilder(new Pose2d(-53, -57, Math.toRadians(45)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58.5, -40, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Action sample2ToBasket = robot.actionBuilder(new Pose2d(-60, -40, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basToSample3 = robot.actionBuilder(new Pose2d(-56, -54, Math.toRadians(45)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-56, -35, Math.toRadians(150)), Math.toRadians(90))
                .build();

        Action sample3ToBasket = robot.actionBuilder(new Pose2d(-56, -35, Math.toRadians(150)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();


        return new SequentialAction(
                arm.actionSetState(ArmStates.GRABBED),          // Move the Arm to scoring Position
                wallToSub ,                                     // Drive to the Sub
                arm.actionClipIt(),                             // Start the clipping action
                arm.actionWaitForState(ArmStates.LOWERING),     // Wait for the the clip to be done
                subToSample1,                                   // Drive to the 1st Sample
                intake.actionIntakeIt(),                        // Start collector to intake sample. sweep if need be
                new ParallelAction(// Start the sample transfer
                        robot.actionSweep(),
                        intake.actionWaitForState(IntakeStates.GOT_SAMPLE)
                ),
                sample1ToBasket,                                // Drive to the basket
                intake.actionWaitForState(IntakeStates.HOME),   // Wait for the transfer to complete
                lift.actionSetState(SAMPLE_HELD),               // Start lift operation
                lift.actionWaitForHomeOrState(LOWERING),              // Wait til the lift is coming down
                basToSample2,
                intake.actionIntakeIt(),                        // Start collector to intake sample. sweep if need be
                new ParallelAction(// Start the sample transfer
                        robot.actionSweep(),
                        intake.actionWaitForState(IntakeStates.GOT_SAMPLE)
                ),
                sample2ToBasket,
                intake.actionWaitForState(IntakeStates.HOME),   // Wait for the transfer to complete
                lift.actionSetState(SAMPLE_HELD),               // Start lift operation
                lift.actionWaitForHomeOrState(LOWERING),
                basToSample3,
                intake.actionIntakeIt(),                        // Start collector to intake sample
                new ParallelAction(// Start the sample transfer
                        robot.actionSweep(),
                        intake.actionWaitForState(IntakeStates.GOT_SAMPLE)
                ),
                sample3ToBasket,
                intake.actionWaitForState(IntakeStates.HOME),   // Wait for the transfer to complete
                lift.actionSetState(SAMPLE_HELD),               // Start lift operation
                lift.actionWaitForHomeOrState(LOWERING)
                );
    }

    // ############################################################################

    @Override
    public void runOpMode()
    {
        Globals.IS_AUTO = true;
        robot = new MecanumDrive(hardwareMap, new Pose2d(0,0,0), this);
        arm.initialize(false);
        lift.initialize(false);
        intake.initialize(true);
        arm.closeClaw();
        autoConfig.initialize();

        selectedAuto = loadSelectedAuto(autoConfig.autoOptions.autoMode);  // build the current auto sequence

        // Wait for driver to press start
        while(opModeInInit()) {
            arm.update();
            lift.update();

            autoConfig.runMenuUI(); //Run menu system
            if (autoConfig.autoOptions.autoMode != lastSelectedAuto) {
                lastSelectedAuto = autoConfig.autoOptions.autoMode;
                selectedAuto = loadSelectedAuto(autoConfig.autoOptions.autoMode);
            }

            telemetry.addLine("\n Touch Play to run Auto");
            telemetry.update();
        }

        // Set GLOBAL flags based on menu choices.
        if (autoConfig.autoOptions.redAlliance )
            Globals.ALLIANCE_COLOR = AllianceColor.RED;
        else
            Globals.ALLIANCE_COLOR = AllianceColor.BLUE;

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Do a count down if these is a delayed start,
            for (int sec = autoConfig.autoOptions.delayStart; sec > 0; sec--) {
                telemetry.addData("AUTO MODE",  "%s", autoConfig.autoArray[autoConfig.autoOptions.autoMode]);
                telemetry.addData("COUNTDOWN",  "%d  %d  %d  %d", sec, sec, sec, sec);
                telemetry.update();
            }

            if (selectedAuto != null) {
                Actions.runBlocking(selectedAuto);
            } else {
                telemetry.addData("AUTO MODE",  "No valid mode selected");
                telemetry.update();
            }
            sleep(1000);
        }

        Globals.LAST_POSE = new Pose2d(0,0, robot.getPose().heading.toDouble()-(Math.PI/2)) ;
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
                sequentialAction = build_Spec_4Sub();
                break;

            case 1:
                sequentialAction = build_Samp_1Bas();
                break;

            case 2:
                sequentialAction = build_Samp_1Bas_Sub();
                break;

            case 3:
                sequentialAction = build_Samp_1Bas_2Net_Sub();
                break;

            case 4:
                sequentialAction = build_Spec_1sub_Sample_1bas();
                break;
        }

        // Run 4 actions simultaniously
        return  new ParallelAction(
                arm.actionUpdate(),
                lift.actionUpdate(),
                intake.actionUpdate(),
                sequentialAction,
                arm.actionUpdateTelemetry()
        );
    }
}
