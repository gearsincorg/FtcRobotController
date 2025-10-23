package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;

import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.*;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {

    public SpindexerSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private DcMotor spinner;
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
    private final double HOME_POWER     = 0.05;
    private final double QUEUEING_POWER = 0.09;
    private final double INTAKE_POWER   = 0.14;
    private final double SHOOTING_POWER = 0.19;

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
        spinner = myOpMode.hardwareMap.get(DcMotor.class, "spinner");
        spinner.setDirection(DcMotor.Direction.FORWARD);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        spindexerAngle = (int)(spinner.getCurrentPosition() / COUNTS_PER_REVOLUTION * 360) % 360;
        currentSlot = spindexerAngle / 120;
        currentSegment = spindexerAngle / 24;

        // only read & update ball color when in range of color sensor
        // currentColor = ArtifactColor.UNKNOWN;
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

            //
            // slotColors[currentSlot] = currentColor;
            // slotColors[currentSlot] = ArtifactColor.PURPLE;

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
    }

    @Override
    public void runStateMachine() {
        switch ((SpindexerStates)currentState) {
            case INIT: {
                spinner.setPower(HOME_POWER);
                setState(HOMING);
                break;
            }

            case HOMING: {
                if (!magnet.getState()) {
                    spinner.setPower(0.0);
                    spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    setState(HOME);
                }
                break;
            }

            case HOME: {
                if (myOpMode.opModeIsActive()) {
                    if (allArtifactsHeld < 3) {
                        spinner.setPower(INTAKE_POWER);
                    }
                    setState(INTAKING);
                }
                break;
            }

            case INTAKING: {
                if (myOpMode.gamepad1.right_bumper || myOpMode.gamepad1.leftBumperWasPressed()) {
                    spinner.setPower(SHOOTING_POWER);
                    setState(SHOOTING);
                } else if (allArtifactsHeld == 3) {
                    spinner.setPower(0.0);
                    setState(QUEUEING);
                } else {
                    spinner.setPower(INTAKE_POWER);
                }
                break;
            }

            case FULL: {
                if (myOpMode.gamepad1.rightBumperWasPressed()) {
                    spinner.setPower(SHOOTING_POWER);
                    setState(SHOOTING);
                }
                break;
            }

            case QUEUEING: {
                if (allArtifactsHeld == 0 ) {
                    spinner.setPower(INTAKE_POWER);
                    setState(INTAKING);
                } else if ((slotColors[currentSlot] != ArtifactColor.UNKNOWN) &&
                        (currentSegment == shootSegments[currentSlot]))   {
                    spinner.setPower(0);
                    setState(QUEUED);
                } else {
                    spinner.setPower(QUEUEING_POWER);
                }
                break;
            }

            case QUEUED: {
                if (myOpMode.gamepad1.right_bumper || myOpMode.gamepad1.leftBumperWasPressed()) {
                    fire.setPosition(FIRE_SHOOT);
                    slotColors[currentSlot] = ArtifactColor.UNKNOWN;
                    spinner.setPower(SHOOTING_POWER);
                    setState(RELOADING);
                }
                break;
            }

            case SHOOTING: {
                if (allArtifactsHeld == 0 ) {
                    spinner.setPower(INTAKE_POWER);
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
                        spinner.setPower(SHOOTING_POWER);
                        setState(SHOOTING);
                    } else {
                        spinner.setPower(QUEUEING_POWER);
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
