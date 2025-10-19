package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.FULL;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.HOME;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.HOMING;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.INIT;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.INTAKING;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.RELOADING;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.SHOOTING;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.STOPPED;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class SpindexerSubsystem {

    private boolean showTelemetry = false;
    private boolean enabled = false;
    private LinearOpMode    myOpMode;
    private ElapsedTime     stateTime       = new ElapsedTime();
    private SpindexerStates currentState    = INIT;

    private DcMotor spinner;
    private DcMotor shooter;
    private Servo fire;

    private NormalizedColorSensor color;
    private DigitalChannel magnet;
    private final double COUNTS_PER_REVOLUTION = 537.5;
    private final double MIN_SATURATION = 0.05;
    private final double GREEN_MIN = 120.0;
    private final double GREEN_MAX = 160.0;
    private final double PURPLE_MIN = 230.0;
    private final double PURPLE_MAX = 300.0;

    private final double HOME_POWER = 0.05;
    private final double INTAKE_POWER = 0.15;
    private final double SHOOTING_POWER = 0.2;

    private final double FIRE_RETRACT = 0.10;
    private final double FIRE_SHOOT   = 0.55;
    private final double FIRE_HOLD_TIME = 0.17;

    private int spindexerAngle = 0;
    private int currentSlot = 0;
    private int currentSegment = 0;
    private int artifactsHeld = 0;

    private ArtifactColor currentColor = ArtifactColor.UNKNOWN;
    private final float[] hsvValues = new float[3];
    private final ArtifactColor[] slotColors = {ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN};
    private final int[] shootSegments = {2,7,12};

    public SpindexerSubsystem(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize the Subsystem by creating hardware devices.
     * @param showTelemetry
     */
    public void init(boolean showTelemetry) {
        this.showTelemetry = showTelemetry;
        this.enabled = true;

        shooter = myOpMode.hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinner = myOpMode.hardwareMap.get(DcMotor.class, "spinner");
        spinner.setDirection(DcMotor.Direction.FORWARD);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fire = myOpMode.hardwareMap.get(Servo.class, "fire");
        fire.setPosition(FIRE_RETRACT);

        color = myOpMode.hardwareMap.get(ColorRangeSensor.class, "color");
        color.setGain(10);

        magnet = myOpMode.hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);
    }

    public void update() {
        // skip if not initialized
        if (!enabled) return;

        currentColor = ArtifactColor.UNKNOWN;

        // Read the spindexer position and detwerming which secment and slot we are in.
        spindexerAngle = (int)(spinner.getCurrentPosition() / COUNTS_PER_REVOLUTION * 360) % 360;
        currentSlot = spindexerAngle / 120;
        currentSegment = spindexerAngle / 24;

        // only read & update ball color when in range of color sensor
        if ((currentSegment == 1) || (currentSegment == 6) || (currentSegment == 11)) {

            NormalizedRGBA colors = color.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            //checking the hue and saturation of the color sensor
            //saturation needs to be high enough use the hue value
            //find which range the hue resides in to decide the color
            if (hsvValues[1] > MIN_SATURATION){
                if ((hsvValues[0] > GREEN_MIN) && (hsvValues[0] < GREEN_MAX)){
                    currentColor = ArtifactColor.GREEN;
                } else if ((hsvValues[0] > PURPLE_MIN) && (hsvValues[0] < PURPLE_MAX)){
                    currentColor = ArtifactColor.PURPLE;
                }
            }

            // slotColors[currentSlot] = currentColor;
            slotColors[currentSlot] = ArtifactColor.PURPLE;

            // count number of slots with balls.
            int ballCount = 0;
            for (int b=0; b < 3; b++){
                if (slotColors[b] != ArtifactColor.UNKNOWN) {
                    ballCount++;
                }
            }
            artifactsHeld = ballCount;
        }

        runStateMachine();

        if (showTelemetry) {
            myOpMode.telemetry.addData("Spindexer", currentState);
            myOpMode.telemetry.addData("Spindexer", "%d Deg, Seg %d Slot %d", spindexerAngle, currentSegment, currentSlot);
            myOpMode.telemetry.addData("Current color", currentColor);
            myOpMode.telemetry.addData("Slots", "%s %s %s", slotColors[0], slotColors[1], slotColors[2]);
            myOpMode.telemetry.addData("magnet", magnet.getState());
        }
    }

    public void runStateMachine() {

        // skip if not initialized
        if (!enabled) return;

        // check the state and look for the required transitions
        switch(currentState) {
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
                    if (artifactsHeld < 3) {
                        spinner.setPower(INTAKE_POWER);
                    }
                    setState(INTAKING);
                } else
                break;
            }

            case INTAKING: {
                if (artifactsHeld == 3) {
                    spinner.setPower(0.0);
                    setState(FULL);
                } else {
                    spinner.setPower(INTAKE_POWER);
                }
                break;
            }

            case STOPPED: {
                break;
            }

            case FULL: {
                if (myOpMode.gamepad1.rightBumperWasPressed()) {
                    spinner.setPower(SHOOTING_POWER);
                    setState(SHOOTING);
                }
                break;
            }

            case SHOOTING: {
                // Do we have something to shoot?
                if ((slotColors[currentSlot] != ArtifactColor.UNKNOWN) &&
                    (currentSegment == shootSegments[currentSlot]))   {
                    fire.setPosition(FIRE_SHOOT);
                    slotColors[currentSlot] = ArtifactColor.UNKNOWN;
                    setState(RELOADING);
                }
                break;
            }

            case RELOADING: {
                if (stateTime.time() > FIRE_HOLD_TIME) {
                    fire.setPosition(FIRE_RETRACT);
                    setState(SHOOTING);
                }
                break;
            }
        }
    }

    public void setState (SpindexerStates newState){
        currentState = newState;
        stateTime.reset();
    }

    //-------------------------------------------------------------------------
    // ACTION  methods
    //-------------------------------------------------------------------------

    public Action actionUpdate(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                update();
                return true;
            }
        };
    }

}
