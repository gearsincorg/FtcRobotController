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
            if (Globals.SHOOTER_AT_SPEED && Globals.TURRET_ON_TARGET && Globals.SPINDEXER_SHOT_CENTERED){
                setLEDMode(LEDMode.SHOOTER_READY);
            } else {
                setLEDMode(LEDMode.SHOOTER_NOT_READY);
            }
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

    void setLEDMode(LEDMode newMode){
        currentLEDMode = newMode;
        prism.loadAnimationsFromArtboard(currentLEDMode.artboardValue);
    }

}
