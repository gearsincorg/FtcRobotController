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
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    static int autoMode = 3;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Configure the starting location for each Auto Mode
        Pose2d  atGoal =  new Pose2d(-58, -45, Math.toRadians(52));
        Pose2d  atWall =  new Pose2d( 62, -16, Math.toRadians(180));
        Pose2d[] autoStartLocations = {atGoal, atWall};

        RoadRunnerBotEntity backBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 60, Math.toRadians(90), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .setDriveTrainType(DriveTrainType.TANK)
                .build();

        RoadRunnerBotEntity frontBot = new  DefaultBotBuilder(meepMeep)
                .setConstraints(40, 60, Math.toRadians(90), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .setDriveTrainType(DriveTrainType.TANK)
                .build();

        DriveShim driveSubsystem = backBot.getDrive();

        //==  BACK BOT  =============================================================
        Action backFirstScore = driveSubsystem.actionBuilder(mirror(autoStartLocations[0]))
                .splineTo(mirror(-24, -12), mirror(0))
                .waitSeconds(2)
                .build();

        Action backCollectPath1 = driveSubsystem.actionBuilder(mirror(-24, -12, 0))
                .splineTo(mirror(-12, -30), mirror(-90))
                .lineToY(mirrorY(-56), new TranslationalVelConstraint(10))
                .waitSeconds(1)
                .build();

        Action backReturnPath1 = driveSubsystem.actionBuilder(mirror(-12, -56, -90))
                .setReversed(true)
                .splineTo(mirror(-24, -12), mirror(-180))
                .waitSeconds(2)
                .build();

        Action backCollectPath2 = driveSubsystem.actionBuilder(mirror(-24, -12, 0))
                .setReversed(false)
                .splineTo(mirror(12, -30), mirror(-90))
                .lineToY(mirrorY(-62), new TranslationalVelConstraint(10))
                .waitSeconds(1)
                .build();

        Action backReturnPath2 = driveSubsystem.actionBuilder(mirror(12, -62, -90))
                .setReversed(true)
                .splineTo(mirror(-24, -12), mirror(-180))
                .waitSeconds(2)
                .build();

        Action backMovePath = driveSubsystem.actionBuilder(mirror(-24, -12, 0))
                .setReversed(false)
                .lineToX(12)
                .build();

        //==  FRONT BOT  =============================================================
        Action frontFirstScore = driveSubsystem.actionBuilder(mirror(autoStartLocations[1]))
                .lineToX(54)
                .waitSeconds(2)
                .build();

        Action frontCollectPath1 = driveSubsystem.actionBuilder(mirror( 54, -16, 180))
                .splineTo(mirror(36, -30), mirror(-90))
                .lineToY(mirrorY(-62), new TranslationalVelConstraint(10))
                .build();

        Action frontReturnPath1 = driveSubsystem.actionBuilder(mirror(36, -62, -90))
                .setReversed(true)
                .splineTo(mirror(54, -16), mirror(90))
                .waitSeconds(2)
                .build();

        Action frontCollectPath2 = driveSubsystem.actionBuilder(mirror(54, -16, -90))
                .setReversed(false)
                .lineToY(mirrorY(-54))
                .splineTo(mirror(58, -62), mirror(-60), new TranslationalVelConstraint(10) )
                .waitSeconds(1)
                .build();

        Action frontReturnPath2 = driveSubsystem.actionBuilder(mirror(58, -62, -60))
                .setReversed(true)
                .splineTo(mirror(54, -16), mirror(90))
                .waitSeconds(2)
                .build();

        Action frontCollectPath3 = driveSubsystem.actionBuilder(mirror(54, -16, -90))
                .setReversed(false)
                .splineTo(mirror(48, -50), mirror(-90))
                .lineToY(mirrorY(-62), new TranslationalVelConstraint(10))
                .waitSeconds(1)
                .build();

        Action frontReturnPath3 = driveSubsystem.actionBuilder(mirror(48, -62, -90))
            .setReversed(true)
            .splineTo(mirror(54, -16), mirror(90))
            .build();

        //===============================================================
        backBot.runAction(new SequentialAction(
                backFirstScore,
                backCollectPath1,
                backReturnPath1,
                backCollectPath2,
                backReturnPath2,
                backMovePath
                ) );

        frontBot.runAction(new SequentialAction(
                frontFirstScore,
                frontCollectPath1,
                frontReturnPath1,
                frontCollectPath2,
                frontReturnPath2,
                frontCollectPath3,
                frontReturnPath3
                ));



        // ---------------------------------------------------------------------------------------
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(backBot)
                .addEntity(frontBot)
                .start();
    }

    private static double mirror(double headingDeg){
        double headingRad = Math.toRadians(headingDeg);
        return headingRad;
    }

    private static double mirrorY(double lineToY){
        return lineToY;
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

