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
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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

    private final double START_Y = -63;
    private final double START_X_SPEC = 14.25;
    private final double START_X_SAMP = -33;

    private final double BASKET_X = -54;
    private final double BASKET_Y = -54;

    // Place all auto builders here!
    //================================================================================================================
    private Action build_2_SpecPreloads_4Sub() {
        robot.setPose(new Pose2d(START_X_SPEC, START_Y, Math.toRadians(90)));

        Action wallToSubPath = robot.actionBuilder(new Pose2d(START_X_SPEC, START_Y, Math.toRadians(90)))
                .splineTo(new Vector2d(4, -32), Math.toRadians(90))
                .build();

        Action subToAllSamplesPath = robot.actionBuilder(new Pose2d(4, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-60))
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -12), Math.toRadians(0))

                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(44, -52), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -24), Math.toRadians(90), new TranslationalVelConstraint(65.0))
                .splineToConstantHeading(new Vector2d(54, -12), Math.toRadians(0))

                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(54, -52), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(54, -24), Math.toRadians(90), new TranslationalVelConstraint(65.0))
                .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))

                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(63, -52), Math.toRadians(-90))
                .build();

        Action samplesToSpecimenPath = robot.actionBuilder(new Pose2d(63, -53, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(41, -44), Math.toRadians(180))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(41, -63), Math.toRadians(-90))
                .build();

        Action specimenToSub2Path = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(130))
                .splineToConstantHeading(new Vector2d(1, -32), Math.toRadians(90))
                .build();

        Action sub2ToSpecimenPath = robot.actionBuilder(new Pose2d(1, -32, Math.toRadians(90)))
                .setTangent(Math.toRadians(-50))
                .splineToConstantHeading(new Vector2d(41, -63), Math.toRadians(-90), new TranslationalVelConstraint(55.0), new ProfileAccelConstraint(-40,180))
                .build();

        Action specimenToSub3Path = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-2, -32), Math.toRadians(90))
                .build();

        Action sub3ToSpecimenPath = robot.actionBuilder(new Pose2d(-2, -32, Math.toRadians(90)))
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(41, -63), Math.toRadians(-90), new TranslationalVelConstraint(55.0), new ProfileAccelConstraint(-40,180))
                .build();

        Action specimenToSub4Path = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(140))
                .splineToConstantHeading(new Vector2d(-5, -32), Math.toRadians(90))
                .build();

        Action sub4ToObservationPath = robot.actionBuilder(new Pose2d(-5, -32, Math.toRadians(90)))
                .setTangent(Math.toRadians(-40))
                .splineToConstantHeading(new Vector2d(50, -56), Math.toRadians(0))
                .build() ;

        //  ######################################################################

        return new SequentialAction(
                // Score Specimen 1 then sweep 2 more
                arm.actionSetState(ArmStates.GRABBING),
                arm.actionWaitForState(ArmStates.GRABBED),
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

    private Action build_1_SpecPrelod_5Sub() {
        robot.setPose(new Pose2d(START_X_SPEC, START_Y, Math.toRadians(90)));

        Action wallToSubPath = robot.actionBuilder(new Pose2d(START_X_SPEC, START_Y, Math.toRadians(90)))
                .splineTo(new Vector2d(4, -32), Math.toRadians(90))
                .build();

        Action subToAllSamplesPath = robot.actionBuilder(new Pose2d(4, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-60))
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -12), Math.toRadians(0))

                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(44, -52), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -24), Math.toRadians(90), new TranslationalVelConstraint(65.0))
                .splineToConstantHeading(new Vector2d(54, -12), Math.toRadians(0))

                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(54, -52), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(54, -24), Math.toRadians(90), new TranslationalVelConstraint(65.0))
                .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))

                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(63, -52), Math.toRadians(-90))
                .build();

        Action samplesToSpecimenPath = robot.actionBuilder(new Pose2d(63, -52, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(53, -52), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(41, -63), Math.toRadians(-90), new TranslationalVelConstraint(25.0))
                .build();

        Action specimenToSubPath2 = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .splineTo(new Vector2d(4, -34), Math.toRadians(135), new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-40,180))
                .build();

        Action subToSpecimenPath2 = robot.actionBuilder(new Pose2d(4, -34, Math.toRadians(135)))
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(41, -63, Math.toRadians(90)), Math.toRadians(-55), new TranslationalVelConstraint(55.0), new ProfileAccelConstraint(-40,180))
                .build();

        Action specimenToSubPath3 = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .splineTo(new Vector2d(3, -34), Math.toRadians(135), new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-40,180))
                .build();

        Action subToSpecimenPath3 = robot.actionBuilder(new Pose2d(3, -34, Math.toRadians(135)))
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(41, -63, Math.toRadians(90)), Math.toRadians(-55), new TranslationalVelConstraint(55.0), new ProfileAccelConstraint(-40,180))
                .build();

        Action specimenToSubPath4 = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .splineTo(new Vector2d(2, -34), Math.toRadians(135), new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-40,180))
                .build();

        Action subToSpecimenPath4 = robot.actionBuilder(new Pose2d(2, -34, Math.toRadians(135)))
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(41, -63, Math.toRadians(90)), Math.toRadians(-55), new TranslationalVelConstraint(55.0), new ProfileAccelConstraint(-40,180))
                .build();

        Action specimenToSubPath5 = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .splineTo(new Vector2d(1, -34), Math.toRadians(135), new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-40,180))
                .build();

        Action subToInspection = robot.actionBuilder(new Pose2d(1, -34, Math.toRadians(135)))
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(90)), 0.0, new TranslationalVelConstraint(65.0), new ProfileAccelConstraint(-100,200))
                .build();


        return new SequentialAction(
                // Score Specimen 1 then sweep 3 more, score 4
                arm.actionSetState(ArmStates.GRABBING),
                arm.actionWaitForState(ArmStates.GRABBED),
                wallToSubPath,
                arm.actionClipIt(),
                arm.actionWaitForState(ArmStates.LOWERING),
                subToAllSamplesPath,

                // Pickup and score Specimen 2
                samplesToSpecimenPath,
                arm.actionSetState(ArmStates.GRABBING),
                arm.actionWaitForState(ArmStates.GRABBED),
                specimenToSubPath2,
                arm.actionClipIt(),
                arm.actionWaitForState(ArmStates.LOWERING),

                // Pickup and score Specimen 3
                subToSpecimenPath2,
                arm.actionSetState(ArmStates.GRABBING),
                arm.actionWaitForState(ArmStates.GRABBED),
                specimenToSubPath3,
                arm.actionClipIt(),
                arm.actionWaitForState(ArmStates.LOWERING),

                // Pickup and score Specimen 4
                subToSpecimenPath3,
                arm.actionSetState(ArmStates.GRABBING),
                arm.actionWaitForState(ArmStates.GRABBED),
                specimenToSubPath4,
                arm.actionClipIt(),
                arm.actionWaitForState(ArmStates.LOWERING),

                // Pickup and score Specimen 5
                subToSpecimenPath4,
                arm.actionSetState(ArmStates.GRABBING),
                arm.actionWaitForState(ArmStates.GRABBED),
                specimenToSubPath5,
                arm.actionClipIt(),
                arm.actionWaitForState(ArmStates.LOWERING),

                // Go Home
                subToInspection
        );
    }

    //================================================================================================================
    private Action build_1_SampPreload_1_Bas() {
        robot.setPose(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)));

        Action wallToBasket = robot.actionBuilder(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basketToSub = robot.actionBuilder(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)))
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
    private Action build_1_SampPreload_1_Bas_2_Net() {
        robot.setPose(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)));

        Action wallToBasket = robot.actionBuilder(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basketToSamples = robot.actionBuilder(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)))
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

    //================================================================================================================
    private Action collectAndScore_3_Bas_Level1() {
        Action sample1ToBasket = robot.actionBuilder(new Pose2d(-48.5, -39, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basToSample2 = robot.actionBuilder(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)))
                .afterTime(0.3, intake.actionLowerIt())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58.5, -39, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Action sample2ToBasket = robot.actionBuilder(new Pose2d(-58.5, -39, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basToSample3 = robot.actionBuilder(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)))
                .afterTime(0.3, intake.actionLowerIt())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-56, -33, Math.toRadians(145)), Math.toRadians(90))
                .build();

        Action sample3ToBasket = robot.actionBuilder(new Pose2d(-56, -33, Math.toRadians(150)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basketToSub = robot.actionBuilder(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)))
                .splineTo(new Vector2d(-46, -44), Math.toRadians(45))
                .splineTo(new Vector2d(-28, -10), Math.toRadians(0))
                .splineTo(new Vector2d(-23, -10), Math.toRadians(0), new TranslationalVelConstraint(5.0))
                .build();

        return new SequentialAction(
                intake.actionIntakeIt(),                        // Start collector to intake sample. sweep if need be
                new ParallelAction(// Start the sample transfer
                        robot.actionSweep(),
                        intake.actionWaitForState(IntakeStates.GOT_SAMPLE)
                ),
                sample1ToBasket,                                // Drive to the basket
                intake.actionWaitForState(IntakeStates.HOME),   // Wait for the transfer to complete
                lift.actionSetState(SAMPLE_HELD),               // Start lift operation
                lift.actionWaitForHomeOrState(LOWERING),        // Wait til the lift is coming down
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
                lift.actionWaitForHomeOrState(LOWERING),

                arm.actionSetState(ArmStates.GRABBED),
                basketToSub
        );
    }

    //==============================================================================================
    private Action build_1_SpecPreload_3_Bas() {
        robot.setPose(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)));

        Action wallToSub = robot.actionBuilder(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)))
                .setTangent(Math.toRadians(35))
                .splineToConstantHeading(new Vector2d(-15, -33), Math.toRadians(90))  // was -15, -31
                .build();

        Action subToSample1 = robot.actionBuilder(new Pose2d(-15, -33, Math.toRadians(90)))
                .afterTime(0.8, intake.actionLowerIt())
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-48.5, -39), Math.toRadians(90), new TranslationalVelConstraint(40.0), new ProfileAccelConstraint(-40,60))
                .build();

        return new SequentialAction(
                arm.actionSetState(ArmStates.GRABBING),
                arm.actionWaitForState(ArmStates.GRABBED),
                wallToSub ,                                     // Drive to the Sub
                arm.actionClipIt(),                             // Start the clipping action
                arm.actionWaitForState(ArmStates.LOWERING),     // Wait for the the clip to be done
                subToSample1,                                   // Drive to the 1st Sample
                collectAndScore_3_Bas_Level1()             // Pickup and score the remaining 3 samples
        );
    }

    //==============================================================================================
    private Action build_1_SampPreload_4_Bas() {
        robot.setPose(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)));

        Action wallToBasket = robot.actionBuilder(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)))
                .afterTime(0.2, lift.actionSetState(SAMPLE_HELD))  // start lifting on the way to the basket.
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basketToSamples = robot.actionBuilder(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)))
                .afterTime(0.1, intake.actionLowerIt())
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48.5, -39, Math.toRadians(90)), Math.toRadians(90))
                .build();

        return new SequentialAction(
                arm.actionClaw(true),
                wallToBasket ,
                // lift.actionSetState(SAMPLE_HELD),            // Start lift operation
                lift.actionWaitForHomeOrState(LOWERING),        // Wait til the lift is coming down
                basketToSamples,
                collectAndScore_3_Bas_Level1()                  // Pickup and score the remaining 3 samples
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
        intake.initialize(false);
        // arm.closeClaw();
        autoConfig.initialize();

        selectedAuto = loadSelectedAuto(autoConfig.autoOptions.autoMode);  // build the current auto sequence

        // Wait for driver to press start
        while(opModeInInit()) {
            arm.update();
            lift.update();
            intake.update();   //  should this be in here ?

            autoConfig.runMenuUI(); //Run menu system
            if (autoConfig.autoOptions.autoMode != lastSelectedAuto) {
                lastSelectedAuto = autoConfig.autoOptions.autoMode;
                selectedAuto = loadSelectedAuto(autoConfig.autoOptions.autoMode);
            }

            // Set GLOBAL flags based on menu choices.
            if (autoConfig.autoOptions.redAlliance )
                Globals.ALLIANCE_COLOR = AllianceColor.RED;
            else
                Globals.ALLIANCE_COLOR = AllianceColor.BLUE;

            intake.setLEDtoAllianceColor();
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
                intake.setLEDtoAllianceColor();
                sleep(500);
                intake.setLEDoff();
                sleep(500);
            }

            if (selectedAuto != null) {
                Actions.runBlocking(selectedAuto);
            } else {
                telemetry.addData("AUTO MODE",  "No valid mode selected");
                telemetry.update();
            }
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
                sequentialAction = build_2_SpecPreloads_4Sub();
                break;

            case 1:
                sequentialAction = build_1_SpecPrelod_5Sub();
                break;

            case 2:
                sequentialAction = build_1_SampPreload_1_Bas();
                break;

            case 3:
                sequentialAction = build_1_SampPreload_1_Bas_2_Net();
                break;

            case 4:
                sequentialAction = build_1_SpecPreload_3_Bas();
                break;

            case 5:
                sequentialAction = build_1_SampPreload_4_Bas();
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
