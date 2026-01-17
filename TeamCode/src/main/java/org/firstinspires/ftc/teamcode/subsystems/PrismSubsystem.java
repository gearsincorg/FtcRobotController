package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;
/*
 *  This Subsystem assumes that Art Boards have been setup in the Prism driver
 *    See LEDMode.java for the function/names of the artboards
 */
public class PrismSubsystem extends SubsystemBase {

    // subsystem devices
    private GoBildaPrismDriver prism;

    public PrismSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // General Subsystem Members

    private LEDMode currentLEDMode = LEDMode.INTAKE_FRONT;

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);  // do not remove
        prism = myOpMode.hardwareMap.get(GoBildaPrismDriver.class,"prism");
        prism.setDefaultBootArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        prism.setStripLength(30);
    }

    @Override
    public void runProcessing() {
        if(myOpMode.opModeInInit()) {
            if (Globals.ALLIANCE_COLOR == AllianceColor.RED) {
                setLEDMode(LEDMode.ALLIANCE_RED);
            } else {
                setLEDMode(LEDMode.ALLIANCE_BLUE);
            }
        } else if (Globals.ROBOT_STATE == RobotStates.INTAKING){
            if (Globals.INTAKE_JAMMED) {
                setLEDMode(LEDMode.INTAKE_JAMMED);
            } else if (Globals.FORWARD_MOTION){
                setLEDMode(LEDMode.INTAKE_FRONT);
            } else {
                setLEDMode(LEDMode.INTAKE_BACK);
            }
        } else {
            if (Globals.SHOOTER_AT_SPEED && Globals.TURRET_ON_TARGET){
                setLEDMode(LEDMode.SHOOTER_READY);
            } else {
                setLEDMode(LEDMode.SHOOTER_NOT_READY);
            }
        }
    }


    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("LED", currentLEDMode);
    }

    public void setLEDMode(LEDMode newMode){
        // Only send the command if the LED mode changes.
        if (newMode != currentLEDMode) {
            currentLEDMode = newMode;
            prism.loadAnimationsFromArtboard(currentLEDMode.artboardValue);
        }
    }


}
