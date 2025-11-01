package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.*;

import org.firstinspires.ftc.teamcode.auxtools.SharedOQ;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {

    public SpindexerSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private Servo fire;
    private Servo spindexer;
    private NormalizedColorSensor frontColorSensor;
    private NormalizedColorSensor backColorSensor;

    // Subsystem Constants
    private final int    OQ_ENCODER_INDEX = 2;
    private final double ENC_TO_DEGREES   = 360 / 8192;
    private final double POSITION_TOLLERANCE = 5;
    private final double COLOR_SENSOR_POSITION_TOLLERANCE = 30;

    // Color match constants
    private final double MIN_SATURATION = 0.1;
    private final float  COLOR_GAIN     = 5;
    private final double GREEN_MIN      = 120.0;
    private final double GREEN_MAX      = 165.0;
    private final double PURPLE_MIN     = 220.0;
    private final double PURPLE_MAX     = 300.0;

    // Flipper Servo positions and times for shooting
    private final double FIRE_RETRACT   = 0.085;
    private final double FIRE_SHOOT     = 0.50;

    private final double ADVANCE_DELAY_TIME = 0.05;
    private final double FIRE_HOLD_TIME = 0.15;

    // Spindexer Servo Positions (in degrees)
    private final double[] SHOOT        = { -90,   0,   90};
    private final double[] INTAKE_FRONT = { -30,  90, -150};
    private final double[] INTAKE_BACK  = { 150, -90,   30};

    // General Subsystem Members
    private double currentAngle    = 0;
    private double targetAngle     = 0;
    private boolean inPosition     = false;
    private boolean nearPosition   = false; 

    private int     currentSlot     = 0;
    private float[] hsvValues = new float[3];
    private int allArtifactsHeld = 0;
    private int greenArtifactsHeld = 0;
    private int purpleArtifactsHeld = 0;

    private ArtifactColor frontColor   = ArtifactColor.UNKNOWN;
    private ArtifactColor backColor    = ArtifactColor.UNKNOWN;
    private ArtifactColor queuedColor  = ArtifactColor.ANY;
    private ArtifactColor[] slotColors = {ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN};

    @Override
    public void init (boolean showTelemetry) {

        super.init(showTelemetry);  // do not remove
        setState(SpindexerStates.INIT);

        // Attach to physical devices and configure them
        fire = myOpMode.hardwareMap.get(Servo.class, "fire");
        fire.setPosition(FIRE_RETRACT);

        spindexer = myOpMode.hardwareMap.get(Servo.class, "spindexer");
        sendSpindexerTo(SHOOT[1]);

        frontColorSensor = myOpMode.hardwareMap.get(ColorRangeSensor.class, "colorFront");
        frontColorSensor.setGain(COLOR_GAIN);

        backColorSensor = myOpMode.hardwareMap.get(ColorRangeSensor.class, "colorBack");
        backColorSensor.setGain(COLOR_GAIN);
    }

    @Override
    public void readSensors() {
        // Read the spindexer position and determine which segment and slot we are in.
        currentAngle   = SharedOQ.OQencoder.positions[OQ_ENCODER_INDEX] * ENC_TO_DEGREES;
        inPosition = Math.abs(targetAngle - currentAngle) < POSITION_TOLLERANCE;
        nearPosition = Math.abs(targetAngle - currentAngle) < COLOR_SENSOR_POSITION_TOLLERANCE;

        // Fix this for two color sensors
        // only read & update ball color when in range of color sensor
        /*
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
        }
        */
    }

    @Override
    public void runStateMachine() {
        switch ((SpindexerStates)currentState) {
            case INIT: {
                sendToShooter(0);
                setState(HOMING);
                break;
            }

            case HOMING: {
                if (inPosition) {
                    setState(HOME);
                }
                break;
            }

            case HOME: {
                if (myOpMode.opModeIsActive()) {
                    setState(INTAKING);
                }
                break;
            }

            case INTAKING: {
                if (allArtifactsHeld == 3) {
                    sendToShooter(0);
                    setState(QUEUEING);
                }
                break;
            }

            case QUEUEING: {
                if (allArtifactsHeld == 0 ) {
                    setState(INTAKING);
                } else if (inPosition)   {
                    setState(READY_TO_SHOOT);
                }
                break;
            }

            case READY_TO_SHOOT: {
                if (myOpMode.gamepad1.right_bumper || myOpMode.gamepad1.leftBumperWasPressed()) {
                    fire.setPosition(FIRE_SHOOT);
                    slotColors[currentSlot] = ArtifactColor.UNKNOWN;
                    setState(SHOOTING);
                }
                break;
            }

            case SHOOTING: {
                // wait for the shot to start  !!  THIS PAUSE MAY NOT BE REQUIRED
                if (timeInState(ADVANCE_DELAY_TIME)) {
                    // Move the spindexer to the next ball if there is one
                    if (allArtifactsHeld == 0) {
                        setState(INTAKING);
                    } else {
                        //  !!!!!!!! advance to the next ball
                        setState(TAKING_SHOT);
                    }
                }
                break;
            }

            case TAKING_SHOT: {
                if (timeInState(FIRE_HOLD_TIME)) {
                    fire.setPosition(FIRE_RETRACT);
                    setState(QUEUEING);
                }
                break;
            }
        }

        // Save current state in Globals for other subsystems
        Globals.SPINDEXER_STATE = (SpindexerStates) currentState;
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("Spin", "%s %.0f -> %.0f %s", currentState, currentAngle, targetAngle, inPosition);
        myOpMode.telemetry.addData("Spin colors", "F=$s, B=$s", frontColor, backColor);
        myOpMode.telemetry.addData("Slots", "%s %s %s", slotColors[0], slotColors[1], slotColors[2]);
    }

    public void sendToFrontIntake(int slot) {
        sendSpindexerTo(INTAKE_FRONT[slot]);
        currentSlot = slot;
    }

    public void sendToBackIntake(int slot) {
        sendSpindexerTo(INTAKE_BACK[slot]);
        currentSlot = slot;
    }

    public void sendToShooter(int slot) {
        sendSpindexerTo(SHOOT[slot]);
        currentSlot = slot;
    }

    public void sendSpindexerTo(double spindexerAngle) {
        targetAngle = spindexerAngle;
        inPosition = false;
        spindexer.setPosition(0.5 - (targetAngle / 150));  // +ve angle turns CCW.
    }

    public void queueColor( ArtifactColor colorToQueue) {
        queuedColor = colorToQueue;
    }
}
