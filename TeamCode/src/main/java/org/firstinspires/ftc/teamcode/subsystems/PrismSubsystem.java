package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class PrismSubsystem extends SubsystemBase {

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

    }

    @Override
    public void runProcessing() {
        if (myOpMode.gamepad1.right_bumper){
            prism.clearAllAnimations();
            prism.insertAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, rainbowSnakes);
        } else if (myOpMode.gamepad1.left_bumper) {
            prism.clearAllAnimations();
            prism.insertAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidBlue);
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
