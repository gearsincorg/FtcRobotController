package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;

import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.*;

import org.firstinspires.ftc.teamcode.auxtools.ServoSpeedController;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {

    public SpindexerSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices

    private ServoSpeedController servoSC;
    private Servo fire;
    private NormalizedColorSensor color;
    private DigitalChannel magnet;

    // Subsystem Constants
    private final double COUNTS_PER_REVOLUTION = 537.5;

    // Color match constants
    private final double MIN_SATURATION = 0.1;
    private final float  COLOR_GAIN     = 5;
    private final double GREEN_MIN      = 120.0;
    private final double GREEN_MAX      = 165.0;
    private final double PURPLE_MIN     = 220.0;
    private final double PURPLE_MAX     = 300.0;

    // Subsystem Speed constants
    private final double HOME_DPS = 0.05 * 360;
    private final double QUEUEING_DPS = 0.09 * 360;
    private final double INTAKE_DPS = 0.14 * 360;
    private final double SHOOTING_DPS = 0.19 * 360;

    // Servo positions
    private final double FIRE_RETRACT   = 0.085;
    private final double FIRE_SHOOT     = 0.50;
    private final double FIRE_HOLD_TIME = 0.15;

    // General Subsystem Members
    private int spindexerAngle = 0;
    private int currentSlot = 0;
    private int currentSegment = 0;
    private float[] hsvValues = new float[3];
    private int[] shootSegments = {2,7,12};
    private int allArtifactsHeld = 0;
    private int greenArtifactsHeld = 0;
    private int purpleArtifactsHeld = 0;

    private ArtifactColor currentColor = ArtifactColor.UNKNOWN;
    private ArtifactColor queuedColor  = ArtifactColor.ANY;
    private ArtifactColor[] slotColors = {ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN};

    @Override
    public void init (boolean showTelemetry) {

        super.init(showTelemetry);  // do not remove
        setState(SpindexerStates.INIT);

        // Attach to physical devices and configure them

        servoSC = new ServoSpeedController(myOpMode, "spindexer", 5, 2);
        servoSC.init(showTelemetry);

        fire = myOpMode.hardwareMap.get(Servo.class, "fire");
        fire.setPosition(FIRE_RETRACT);

        color = myOpMode.hardwareMap.get(ColorRangeSensor.class, "color");
        color.setGain(COLOR_GAIN);

        magnet = myOpMode.hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void readSensors() {
        // Read the spindexer position and detwerming which secment and slot we are in.
        spindexerAngle = (int)servoSC.getAng();
        currentSlot = spindexerAngle / 120;
        currentSegment = spindexerAngle / 24;

        // only read & update ball color when in range of color sensor
        if ((currentSegment == 0) || (currentSegment == 5) || (currentSegment == 10)) {

            NormalizedRGBA colors = color.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            //checking the hue and saturation of the color sensor
            //saturation needs to be high enough use the hue value
            //find which range the hue resides in to decide the color
            if (hsvValues[1] > MIN_SATURATION) {
                if ((hsvValues[0] > GREEN_MIN) && (hsvValues[0] < GREEN_MAX)) {
                    currentColor = ArtifactColor.GREEN;
                    slotColors[currentSlot] = currentColor;
                } else if ((hsvValues[0] > PURPLE_MIN) && (hsvValues[0] < PURPLE_MAX)) {
                    currentColor = ArtifactColor.PURPLE;
                    slotColors[currentSlot] = currentColor;
                }
            }

            // count number of slots with balls.
            int purpleCount = 0;
            int greenCount = 0;
            for (int b = 0; b < 3; b++) {
                if (slotColors[b] == ArtifactColor.GREEN) {
                    greenCount++;
                } else if (slotColors[b] == ArtifactColor.PURPLE) {
                    purpleCount++;
                }
            }

            allArtifactsHeld    = greenCount + purpleCount;
            greenArtifactsHeld  = greenCount;
            purpleArtifactsHeld = purpleCount;

            servoSC.update();
        }
    }

    @Override
    public void runStateMachine() {
        switch ((SpindexerStates)currentState) {
            case INIT: {
                servoSC.setVelTarget(HOME_DPS);
                setState(HOMING);
                break;
            }

            case HOMING: {
                if (!magnet.getState()) {
                    servoSC.stop();
                    setState(HOME);
                }
                break;
            }

            case HOME: {
                if (myOpMode.opModeIsActive()) {
                    if (allArtifactsHeld < 3) {
                        servoSC.setVelTarget(INTAKE_DPS);
                    }
                    setState(INTAKING);
                }
                break;
            }

            case INTAKING: {
                if (myOpMode.gamepad1.right_bumper || myOpMode.gamepad1.leftBumperWasPressed()) {
                    servoSC.setVelTarget(SHOOTING_DPS);
                    setState(SHOOTING);
                } else if (allArtifactsHeld == 3) {
                    servoSC.stop();
                    setState(QUEUEING);
                } else {
                    servoSC.setVelTarget(INTAKE_DPS);
                }
                break;
            }

            case FULL: {
                if (myOpMode.gamepad1.rightBumperWasPressed()) {
                    servoSC.setVelTarget(SHOOTING_DPS);
                    setState(SHOOTING);
                }
                break;
            }

            case QUEUEING: {
                if (allArtifactsHeld == 0 ) {
                    servoSC.setVelTarget(INTAKE_DPS);
                    setState(INTAKING);
                } else if ((slotColors[currentSlot] != ArtifactColor.UNKNOWN) &&
                        (currentSegment == shootSegments[currentSlot]))   {
                    servoSC.stop();
                    setState(QUEUED);
                } else {
                    servoSC.setVelTarget(QUEUEING_DPS);
                }
                break;
            }

            case QUEUED: {
                if (myOpMode.gamepad1.right_bumper || myOpMode.gamepad1.leftBumperWasPressed()) {
                    fire.setPosition(FIRE_SHOOT);
                    slotColors[currentSlot] = ArtifactColor.UNKNOWN;
                    servoSC.setVelTarget(SHOOTING_DPS);
                    setState(RELOADING);
                }
                break;
            }

            case SHOOTING: {
                if (allArtifactsHeld == 0 ) {
                    servoSC.setVelTarget(INTAKE_DPS);
                    setState(INTAKING);
                } else if ((slotColors[currentSlot] != ArtifactColor.UNKNOWN) &&
                        (currentSegment == shootSegments[currentSlot]))   {
                    fire.setPosition(FIRE_SHOOT);
                    slotColors[currentSlot] = ArtifactColor.UNKNOWN;
                    setState(RELOADING);
                }
                break;
            }

            case RELOADING: {
                if (timeInState(FIRE_HOLD_TIME)) {
                    fire.setPosition(FIRE_RETRACT);
                    if (myOpMode.gamepad1.right_bumper) {
                        servoSC.setVelTarget(SHOOTING_DPS);
                        setState(SHOOTING);
                    } else {
                        servoSC.setVelTarget(QUEUEING_DPS);
                        setState(QUEUEING);
                    }
                }
                break;
            }

        }
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("Spindexer", currentState);
        myOpMode.telemetry.addData("Spindexer", "%d Deg, Seg %d Slot %d", spindexerAngle, currentSegment, currentSlot);
        myOpMode.telemetry.addData("Current color", currentColor);
        myOpMode.telemetry.addData("Slots", "%s %s %s", slotColors[0], slotColors[1], slotColors[2]);
        myOpMode.telemetry.addData("magnet", magnet.getState());
    }

    public void queueColor( ArtifactColor colorToQueue) {
        queuedColor = colorToQueue;
    }
}
