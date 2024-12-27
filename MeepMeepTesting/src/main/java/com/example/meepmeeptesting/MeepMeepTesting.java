package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {

        final double START_Y = -62;
        final double START_X_SPEC = 15;
        final double START_X_SAMP = -15;

        MeepMeep meepMeep = new MeepMeep(640);

        RoadRunnerBotEntity specimenBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 150, Math.toRadians(180), Math.toRadians(360), 15)
                .build();

        RoadRunnerBotEntity specimenToSubBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(360), 15)
                .build();

        DriveShim robot = specimenBot.getDrive();

        RoadRunnerBotEntity basketBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // ===============================================================================================
        //  build Specimen trajectories

        //build trajectories
        Action wallToSubPath = robot.actionBuilder(new Pose2d(15, -63, Math.toRadians(90)))
                .splineTo(new Vector2d(4, -32), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();

        Action subToAllSamplesPath = robot.actionBuilder(new Pose2d(4, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-60))
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -12), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(44, -24), Math.toRadians(-90), new TranslationalVelConstraint(30.0))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(44, -52), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(54, -12), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(54, -24), Math.toRadians(-90), new TranslationalVelConstraint(30.0))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(54, -52), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(54, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(63, -12), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                //.splineToConstantHeading(new Vector2d(63, -24), Math.toRadians(-90), new TranslationalVelConstraint(30.0))
                .splineToConstantHeading(new Vector2d(63, -52), Math.toRadians(-90))
                .build();

        Action samplesToSpecimenPath = robot.actionBuilder(new Pose2d(63, -52, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(53, -52), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(41, -63), Math.toRadians(-90), new TranslationalVelConstraint(30.0))
                .waitSeconds(0.4)
                .build()
                ;

        Action specimenToSubPath = robot.actionBuilder(new Pose2d(41, -63, Math.toRadians(90)))
                .splineTo(new Vector2d(4, -36), Math.toRadians(135), new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-40,180))
                .waitSeconds(0.4)
                .build();

        Action subToSpecimenPath = robot.actionBuilder(new Pose2d(4, -36, Math.toRadians(135)))
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(41, -63, Math.toRadians(90)), Math.toRadians(-55), new TranslationalVelConstraint(55.0), new ProfileAccelConstraint(-40,180))

                .waitSeconds(0.4)
                .build();

        Action subToInspection = robot.actionBuilder(new Pose2d(0, -36, Math.toRadians(140)))
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(90)), 0.0, new TranslationalVelConstraint(65.0), new ProfileAccelConstraint(-100,200))
                .build();


        // ===============================================================================================
        // Build Basket Trajectories

        Action wallToBasket = robot.actionBuilder(new Pose2d(-32, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-54, -58, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build()  ;

        Action basketToSub = robot.actionBuilder(new Pose2d(-53, -57, Math.toRadians(45)))
                .splineTo(new Vector2d(-43, -47), Math.toRadians(45))
                .splineTo(new Vector2d(-28, -10), Math.toRadians(0))
                .splineTo(new Vector2d(-23, -10), Math.toRadians(0), new TranslationalVelConstraint(5.0))
                .build();

        // ===============================================================================================
        specimenBot.runAction(new SequentialAction(
                // Score Specimen 1 then sweep 3 more
                wallToSubPath,
                subToAllSamplesPath,
                // Pickup and score Specimen 2
                samplesToSpecimenPath,
                specimenToSubPath,
                // Pickup and score Specimen 3
                subToSpecimenPath,
                specimenToSubPath,
                // Pickup and score Specimen 4
                subToSpecimenPath,
                specimenToSubPath,
                // Pickup and score Specimen 5
                subToSpecimenPath,
                specimenToSubPath,

                // Go Home
                subToInspection
        ));

        //====================================================================================

        Action wallToSub = robot.actionBuilder(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-15, -31), Math.toRadians(90))
                .build();

        Action subToSample1 = robot.actionBuilder(new Pose2d(-15, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-49, -40), Math.toRadians(90))
                .build();

        Action sample1ToBasket = robot.actionBuilder(new Pose2d(-49, -40, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basToSample2 = robot.actionBuilder(new Pose2d(-56, -54, Math.toRadians(45)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-60, -40, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Action sample2ToBasket = robot.actionBuilder(new Pose2d(-60, -40, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basToSample3 = robot.actionBuilder(new Pose2d(-56, -54, Math.toRadians(45)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-56, -32, Math.toRadians(150)), Math.toRadians(90))
                .build();

        Action sample3ToBasket = robot.actionBuilder(new Pose2d(-56, -32, Math.toRadians(150)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();


        specimenToSubBot.runAction(new SequentialAction(
                wallToSub,
                subToSample1,
                sample1ToBasket,
                basToSample2,
                sample2ToBasket,
                basToSample3,
                sample3ToBasket
        ));

        basketBot.runAction(new SequentialAction(
                   wallToBasket,
                   basketToSub
                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(specimenToSubBot)
                .addEntity(specimenBot)
                //.addEntity(basketBot)
                .start();
    }
}