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
    private static boolean redAlliance = false;

    // Configure the starting location for each Auto Mode
    private static Pose2d  atOrigin =  new Pose2d(0, 0, 0);
    private static Pose2d  atGoal =  new Pose2d(-58, -45, Math.toRadians(52));
    private static Pose2d  atWall =  new Pose2d( 62, -16, Math.toRadians(180));
    private static Pose2d[] autoStartLocations = {atGoal, atWall, atOrigin};

    private static DriveShim driveSubsystem;

    // ==========================================================================================
    // Place all Back-Side paths here!
    // ==========================================================================================

    private static Action goalScorePreloads() {
        return driveSubsystem.actionBuilder(mirror(autoStartLocations[0]))
                .lineToY(mirrorY(-20))
                .waitSeconds(0.25)
                .build();
    }

    private static Action turnToZero() {
        return driveSubsystem.actionBuilder(mirror(-39, -20, 52))
                .turnTo(mirror(0))
                .build();
    }

    private static Action backCollectRow1() {
        return driveSubsystem.actionBuilder(mirror(-39, -20, 0))
                .splineTo(mirror(-12, -42), mirror(-90), new TranslationalVelConstraint(20))
                .lineToY(mirrorY(-56), new TranslationalVelConstraint(12))
                .build();
    }

    private static Action backReleaseAfterRow1() {
        return driveSubsystem.actionBuilder(mirror(-12, -56, -90))
                .setReversed(true)
                .splineTo(mirror(-6, -36), mirror(90))
                .setReversed(false)
                .splineTo(mirror(-4, -56), mirror(-90))
                .waitSeconds(1)
                .build();
    }

    private static Action backReturnAfterRelease() {
        return driveSubsystem.actionBuilder(mirror(-4, -56, -90))
                .setReversed(true)
                .splineTo(mirror(-12, -20), mirror(90))
                .build();
    }

    private static Action backReturnRow1Final() {
        return driveSubsystem.actionBuilder(mirror(-12, -56, -90))
                .setReversed(true)
                .splineTo(mirror(-40, -24), mirror(150))
                .build();
    }

    private static Action backReturnRow1() {
        return driveSubsystem.actionBuilder(mirror(-12, -56, -90))
                .setReversed(true)
                .lineToY(mirrorY(-20))
                .build();
    }

    private static Action backCollectRow2() {
        return driveSubsystem.actionBuilder(mirror(-12, -20, -90))
                .setReversed(false)
                .turnTo(mirror(0))
                .splineTo(mirror(12, -42), mirror(-90), new TranslationalVelConstraint(20))
                .lineToY(mirrorY(-62), new TranslationalVelConstraint(12))
                .build();
    }

    private static Action backReturnRow2Final() {
        return driveSubsystem.actionBuilder(mirror(12, -56, -90))
                .setReversed(true)
                .lineToY(mirrorY(-48))
                .splineTo(mirror(-12, -24), mirror(180))
                .lineToX(-40)
                .build();
    }

    private static Action backReturnRow2() {
        return driveSubsystem.actionBuilder(mirror(12, -56, -90))
                .setReversed(true)
                .lineToY(mirrorY(-44))
                .splineTo(mirror(-12, -20), mirror(180))
                .lineToX(-12)
                .build();
    }

    private static Action backCollectRow3() {
        return driveSubsystem.actionBuilder(mirror(-12, -20, 0))
                .setReversed(false)
                .lineToX(12)
                .splineTo(mirror(36, -42), mirror(-90), new TranslationalVelConstraint(20))
                .lineToY(mirrorY(-62), new TranslationalVelConstraint(12))
                .waitSeconds(1)
                .build();
    }

    private static Action backReturnRow3Final() {
        return driveSubsystem.actionBuilder(mirror(36, -62, -90))
                .lineToY(mirrorY(-44))
                .splineTo(mirror(12, -20), mirror(180))
                .lineToX(-36)
                .build();
    }

    // ==========================================================================================
    // Place all Front-side paths here!
    // ==========================================================================================

    private static Action frontScorePreloads() {
        return driveSubsystem.actionBuilder(mirror(autoStartLocations[1]))
                .lineToX(54)
                .build();
    }

    private static Action frontLeave() {
        return  driveSubsystem.actionBuilder(mirror(54, -13, 180))
                .lineToX(36)
                .build();
    }

    private static Action frontCollectRow3() {
        return driveSubsystem.actionBuilder(mirror(54, -16, 180))
                .splineTo(mirror(36, -30), mirror(-90))
                .lineToY(mirrorY(-64), new TranslationalVelConstraint(12))
                .build();
    }

    private static Action frontReturnRow3() {
        return driveSubsystem.actionBuilder(mirror(36, -62, -90))
                .setReversed(true)
                .splineTo(mirror(54, -16), mirror(90))
                .build();
    }

    private static Action frontTurnToMinus90() {
        return driveSubsystem.actionBuilder(mirror(54, -16, 180))
                .turnTo(mirror(-90))
                .build();
    }

    private static Action frontCollectCycle() {
        return driveSubsystem.actionBuilder(mirror(54, -16, -90))
                .lineToY(mirrorY(-56))
                .lineToY(mirrorY(-62), new TranslationalVelConstraint(12))
                .build();
    }

    private static Action frontReturnCycle() {
        return driveSubsystem.actionBuilder(mirror(54, -62, -90))
                .setReversed(true)
                .lineToY(mirrorY(-18))
                .lineToY(mirrorY(-16), new TranslationalVelConstraint(12))
                .waitSeconds(0.33)
                .build();
    }

    private static Action frontCollectMoreCycle() {
        return driveSubsystem.actionBuilder(mirror(54, -16, -90))
                .splineTo(mirror(38, -56), mirror(-90))
                .lineToY(mirrorY(-62), new TranslationalVelConstraint(12))
                .build();
    }

    private static Action frontReturnMoreCycle() {
        return driveSubsystem.actionBuilder(mirror(38, -62, -90))
                .setReversed(true)
                .splineTo(mirror(54, -18), mirror(90))
                .lineToY(mirrorY(-16), new TranslationalVelConstraint(12))
                .waitSeconds(0.33)
                .build();
    }

    // ==========================================================================================

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity Bot1= new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 100, Math.toRadians(180), Math.toRadians(360), 15)
            .setDimensions(16,16)
            .setDriveTrainType(DriveTrainType.TANK)
            .build();

        RoadRunnerBotEntity Bot2 = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 100, Math.toRadians(180), Math.toRadians(360), 15)
            .setDimensions(16,16)
            .setDriveTrainType(DriveTrainType.TANK)
            .build();

        RoadRunnerBotEntity Bot3 = new  DefaultBotBuilder(meepMeep)
            .setConstraints(60, 100, Math.toRadians(90), Math.toRadians(180), 15)
            .setDimensions(16,16)
            .setDriveTrainType(DriveTrainType.TANK)
            .build();

        RoadRunnerBotEntity Bot4 = new  DefaultBotBuilder(meepMeep)
            .setConstraints(60, 100, Math.toRadians(90), Math.toRadians(180), 15)
            .setDimensions(16,16)
            .setDriveTrainType(DriveTrainType.TANK)
            .build();

        driveSubsystem = Bot1.getDrive();

        //===============================================================
        Bot1.runAction(new SequentialAction(
                goalScorePreloads(),
                turnToZero(),
                backCollectRow1(),
                backReturnRow1(),
                backCollectRow2(),
                backReturnRow2Final()
        ) );

        Bot2.runAction(new SequentialAction(
                frontScorePreloads(),
                frontCollectRow3(),
                frontReturnRow3(),
                frontCollectCycle(),
                frontReturnCycle(),
                frontCollectMoreCycle(),
                frontReturnMoreCycle(),
                frontCollectCycle()
        ));

        redAlliance = true;

        Bot3.runAction(new SequentialAction(
                goalScorePreloads(),
                backCollectRow1(),
                backReleaseAfterRow1(),
                backReturnAfterRelease(),
                backCollectRow2(),
                backReturnRow2Final()
        ));

        Bot4.runAction(new SequentialAction(
                frontScorePreloads(),
                frontTurnToMinus90(),
                frontCollectCycle(),
                frontReturnCycle(),
                frontCollectCycle(),
                frontReturnCycle(),
                frontCollectCycle(),
                frontReturnCycle(),
                frontCollectCycle()
        ) );


        //======================================================================================================


        // ---------------------------------------------------------------------------------------
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Bot1)
                .addEntity(Bot2)
                .addEntity(Bot3)
                .addEntity(Bot4)
                .start();
    }

    // =====================================================================
    private static double mirror(double headingDeg){
        double headingRad = Math.toRadians(headingDeg);
        if (redAlliance){
            headingRad = -headingRad;
        }
        return headingRad;
    }

    private static double mirrorY(double lineToY){
        if (redAlliance){
            lineToY = -lineToY;
        }
        return lineToY;
    }

    private static Vector2d mirror(Vector2d positionXY){
        if (redAlliance){
            positionXY = new Vector2d(positionXY.x, -positionXY.y);
        }
        return positionXY;
    }

    private static Vector2d mirror(double x, double y){
        if (redAlliance){
            return new Vector2d(x, -y);
        }else{
            return new Vector2d(x, y);
        }
    }

    private static Pose2d mirror(Pose2d pose){
        if (redAlliance){
            pose = new Pose2d(pose.position.x, -pose.position.y, -pose.heading.toDouble());
        }
        return pose;
    }

    private static Pose2d mirror(double x, double y, double headingDeg){
        if (redAlliance){
            return new Pose2d(x, -y, Math.toRadians(-headingDeg));
        } else {
            return new Pose2d(x, y, Math.toRadians(headingDeg));
        }
    }

}

