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
        final double BASKET_X = -54;
        final double BASKET_Y = -54;

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

        // ===============================================================================================
        // Build Basket Trajectories





        // ===============================================================================================
        specimenBot.runAction(new SequentialAction(
                // Score Specimen 1 then sweep 3 more
                wallToSubPath,
                subToAllSamplesPath,
                // Pickup and score Specimen 2
                samplesToSpecimenPath,
                specimenToSubPath2,
                subToSpecimenPath2,
                specimenToSubPath3,
                subToSpecimenPath3,
                specimenToSubPath4,
                subToSpecimenPath4,
                specimenToSubPath5,
                // Go Home
                subToInspection
        ));

        //====================================================================================


        Action wallToBasket = robot.actionBuilder(new Pose2d(START_X_SAMP, START_Y, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basketToSamples = robot.actionBuilder(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48.5, -39, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Action sample1ToBasket = robot.actionBuilder(new Pose2d(-48.5, -39, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basToSample2 = robot.actionBuilder(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58.5, -39, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Action sample2ToBasket = robot.actionBuilder(new Pose2d(-58.5, -39, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)), Math.toRadians(-135), new TranslationalVelConstraint(15.0))
                .build();

        Action basToSample3 = robot.actionBuilder(new Pose2d(BASKET_X, BASKET_Y, Math.toRadians(45)))
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

        //specimenToSubBot.runAction(new SequentialAction(
        //        wallToSub,
          //      subToSample1,
            //    sample1ToBasket,
              //  basToSample2,
                //sample2ToBasket,
                //basToSample3,
                //sample3ToBasket
        //));

        basketBot.runAction(new SequentialAction(
                   wallToBasket,
                   basketToSamples,
                   sample1ToBasket,
                   basToSample2,
                   sample2ToBasket,
                   basToSample3,
                   sample3ToBasket,
                   basketToSub
                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(specimenToSubBot)
                .addEntity(specimenBot)
                .addEntity(basketBot)
                .start();
    }
}