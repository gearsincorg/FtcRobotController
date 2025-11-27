package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    static int autoMode = 3;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Configure the starting location for each Auto Mode
        Pose2d  atGoal =  new Pose2d(-58, -45, Math.toRadians(52));
        Pose2d[] autoStartLocations = {atGoal, atGoal, atGoal, atGoal};

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 50, Math.toRadians(180), Math.toRadians(360), 15)
                .build();

        // ---------------------------------------------------------------------------------------

        Action drivePath = myBot.getDrive().actionBuilder(mirror(autoStartLocations[autoMode]))
                .splineTo(mirror(-48, -32), mirror(45))
                .build();

        Action collectPath = myBot.getDrive().actionBuilder(mirror(-48, -32, 45))
                .splineTo(mirror(-12, -24), mirror(-89))
                .splineTo (mirror(-12, -36), mirror(-90), new TranslationalVelConstraint(5))
                .build();


        myBot.runAction(new SequentialAction(
                drivePath,
                collectPath) );

        // ---------------------------------------------------------------------------------------
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    private static double mirror(double headingDeg){
        double headingRad = Math.toRadians(headingDeg);
        return headingRad;
    }

    private  static Vector2d mirror(Vector2d positionXY){
        return positionXY;
    }

    private  static Vector2d mirror(double x, double y){
        return new Vector2d(x, y);
    }

    private  static Pose2d mirror(Pose2d pose){
        return pose;
    }

    private  static Pose2d mirror(double x, double y, double headingDeg){
        return new Pose2d(x, y, Math.toRadians(headingDeg));
    }
}

