package org.firstinspires.ftc.teamcode.auxtools;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.subsystems.PrismSubsystem;

@TeleOp(name="Dustin Artboard Loader", group="Linear OpMode")
//@Disabled

public class ProgramArtboards extends LinearOpMode{
    GoBildaPrismDriver prism;

    @Override public void runOpMode(){
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
        prism.setStripLength(30);  //  0-11, 12-17, 18-29
        prism.setDefaultBootArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        prism.enableDefaultBootArtboard(true);

        // ================================================================================================
        // build all the animations
        // To see what each Artboard should be doing, look at LEDMode.java
        // ================================================================================================
        telemetry.addLine("Creating Animations");
        telemetry.update();

        // setup the animations
        PrismAnimations.Solid  cameraLight = new PrismAnimations.Solid(Color.WHITE);
        cameraLight.setStartIndex(12);
        cameraLight.setStopIndex(17);
        cameraLight.setBrightness(50);

        PrismAnimations.DroidScan  teamColors = new PrismAnimations.DroidScan();
        teamColors.setStartIndex(0);
        teamColors.setStopIndex(11);
        teamColors.setBrightness(50);
        teamColors.setPrimaryColor(Color.YELLOW);
        teamColors.setDroidScanStyle(PrismAnimations.DroidScan.DroidScanStyle.BOTH_TAIL);
        teamColors.setTrailWidth(2);
        teamColors.setSpeed(0.1f);

        PrismAnimations.Snakes  redAlliance = new PrismAnimations.Snakes();
        redAlliance.setStartIndex(0);
        redAlliance.setStopIndex(11);
        redAlliance.setBrightness(50);
        redAlliance.setColors(Color.RED);

        PrismAnimations.Snakes  blueAlliance = new PrismAnimations.Snakes();
        blueAlliance.setStartIndex(0);
        blueAlliance.setStopIndex(11);
        blueAlliance.setBrightness(50);
        blueAlliance.setColors(Color.BLUE);

        PrismAnimations.Solid   intakeFront = new PrismAnimations.Solid(Color.GREEN);
        intakeFront.setIndexes(0, 11);
        intakeFront.setBrightness(50);

        PrismAnimations.Solid   intakeBack = new PrismAnimations.Solid(Color.GREEN);
        intakeBack.setIndexes(18, 29);
        intakeBack.setBrightness(50);

        PrismAnimations.Blink    jammed = new PrismAnimations.Blink(Color.RED);
        jammed.setIndexes(0, 29);
        jammed.setBrightness(100);
        jammed.setSecondaryColor(Color.TRANSPARENT);
        jammed.setPrimaryColorPeriod(20);
        jammed.setPeriod(70);

        PrismAnimations.Solid    shooterReady = new PrismAnimations.Solid(Color.YELLOW);
        shooterReady.setIndexes(18, 29);
        shooterReady.setBrightness(50);

        PrismAnimations.Blink    shooterNotReady = new PrismAnimations.Blink(Color.PURPLE);
        shooterNotReady.setIndexes(0, 11);
        shooterNotReady.setBrightness(50);
        shooterNotReady.setSecondaryColor(Color.TRANSPARENT);
        shooterNotReady.setPrimaryColorPeriod(20);
        shooterNotReady.setPeriod(70);

        // set up the rest of the snake pattern

        //  keep going

        // ================================================================================================
        // load the animations into each artboard.
        // To see what each Artboard should be doing, look at LEDMode.java
        // ================================================================================================
        telemetry.addLine("Loading Artboards");
        telemetry.update();

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, teamColors);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, cameraLight);
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        sleep(1000);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, redAlliance);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, cameraLight);
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blueAlliance);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, cameraLight);
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, intakeFront);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, cameraLight);
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, intakeBack);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, cameraLight);
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_4);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, shooterReady);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, cameraLight);
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_5);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, shooterNotReady);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, cameraLight);
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_6);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, jammed);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, cameraLight);
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);

        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);

        while (opModeInInit()) {
            if (gamepad1.dpadDownWasPressed()) {
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
            } else if (gamepad1.dpadLeftWasPressed()) {
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
            } else if (gamepad1.dpadUpWasPressed()) {
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
            } else if (gamepad1.dpadRightWasPressed()) {
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);
            } else if (gamepad1.aWasPressed()) {
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_4);
            } else if (gamepad1.bWasPressed()) {
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_5);
            } else if (gamepad1.xWasPressed()) {
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_6);
            } else if (gamepad1.yWasPressed()) {
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
            }
        }

        // Keep going
        
        telemetry.addLine("Loading Complete");
        telemetry.update();
    }
}
