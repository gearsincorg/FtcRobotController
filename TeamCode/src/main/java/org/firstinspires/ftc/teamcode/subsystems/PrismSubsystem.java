package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class PrismSubsystem extends SubsystemBase {

    //THING TO DO
    /*
    ALIANCE COLORS
    GREEN FOR COLLECTING
    yellow flashing for collecting
    red = bad
     */

    // subsystem devices
    private GoBildaPrismDriver prism;
    PrismAnimations.Solid solidBlue = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();

    // Subsystem Constants

    // Subsystem Speed/Power constants

    // Servo positions

    // General Subsystem Members

    public PrismSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices

    // Subsystem Constants

    // Subsystem Speed/Power constants

    // Servo positions

    // General Subsystem Members

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);  // do not remove
        prism = myOpMode.hardwareMap.get(GoBildaPrismDriver.class,"prism");
        prism.setDefaultBootArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        prism.setStripLength(12);
    }

    @Override
    public void runProcessing() {
        if (myOpMode.gamepad1.rightBumperWasPressed()){
            if (prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, rainbowSnakes)) {
                myOpMode.telemetry.addLine("rainbow OK");
            } else {
                myOpMode.telemetry.addLine("rainbow FAIL");
            }
            myOpMode.telemetry.update();
        } else if (myOpMode.gamepad1.leftBumperWasPressed()) {
            if (prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidBlue)) {
                myOpMode.telemetry.addLine("blue OK");
            } else {
                myOpMode.telemetry.addLine("blue FAIL");
            }
            myOpMode.telemetry.update();
        }
    }

    @Override
    public void readSensors() {

    }

    @Override
    public void runStateMachine() {

    }

    @Override
    public void showStatus() {

    }

}
