package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(640);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        DriveShim robot = myBot.getDrive();

        // ===============================================================================================

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
                .splineToConstantHeading(new Vector2d(44, -45), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(53, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(53, -50), Math.toRadians(-90))
                .build()
                ;

        Action samplesToSpecimenPath = robot.actionBuilder(new Pose2d(53, -50, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(30, -63), Math.toRadians(-90))
                .build()
                ;

        Action specimenToSub2Path = robot.actionBuilder(new Pose2d(30, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(145))
                .splineToConstantHeading(new Vector2d(0, -31), Math.toRadians(90))
                .build()
                ;

        Action sub2ToSpecimenPath = robot.actionBuilder(new Pose2d(0, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-35))
                .splineToConstantHeading(new Vector2d(30, -63), Math.toRadians(-90))
                .build()
                ;

        Action specimenToSub3Path = robot.actionBuilder(new Pose2d(30, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(new Vector2d(-4, -31), Math.toRadians(90))
                .build()
                ;

        Action sub3ToSpecimenPath = robot.actionBuilder(new Pose2d(-4, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-30))
                .splineToConstantHeading(new Vector2d(30, -63), Math.toRadians(-90))
                .build()
                ;

        Action specimenToSub4Path = robot.actionBuilder(new Pose2d(30, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(155))
                .splineToConstantHeading(new Vector2d(-8, -31), Math.toRadians(90))
                .build()
                ;

        Action sub4ToObservationPath = robot.actionBuilder(new Pose2d(-8, -31, Math.toRadians(90)))
                .setTangent(Math.toRadians(-30))
                .splineToConstantHeading(new Vector2d(50, -56), Math.toRadians(0))
                .build()
                ;
        // ===============================================================================================

        myBot.runAction(new SequentialAction(
                // Score Specimen 1 then sweep 2 more
                wallToSubPath,
                subToAllSamplesPath,
                // Pickup and score Specimen 2
                samplesToSpecimenPath,
                specimenToSub2Path,
                // Pickup and score Specimen 3
                sub2ToSpecimenPath,
                specimenToSub3Path,
                // Pickup and score Specimen 3
                sub3ToSpecimenPath,
                specimenToSub4Path,
                // Go to park
                sub4ToObservationPath

        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}