package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.SampleColor.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.SampleColor.NONE;
import static org.firstinspires.ftc.teamcode.subsystems.SampleColor.RED;
import static org.firstinspires.ftc.teamcode.subsystems.SampleColor.YELLOW;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {

    // Standard SubSystem Members:
    private LinearOpMode myOpMode;
    private boolean      showTelemetry     = false;
    private ElapsedTime  stateTime = new ElapsedTime();
    private IntakeStates currentState = IntakeStates.INIT;

    // Constants
    private final double LEFT_LEVER_IN = 0.50;
    private final double RIGHT_LEVER_IN = 0.50;
    private final double LEFT_LEVER_OUT = 0.62;
    private final double RIGHT_LEVER_OUT = 0.42;
    private final double INTAKE = 1;
    private final double EJECT = -1;
    private final double OFF = 0;
    private final double WRIST_IN = 0.4;
    private final double WRIST_OUT = 0.68;
    private final double WRIST_DOWN = 0.87;

    // public members
    public boolean     gotSample   = false;
    public SampleColor sampleColor = NONE;

    //declaring servos for the intake
    private Servo leftLever;
    private Servo rightLever;
    private CRServo leftWheel;
    private CRServo rightWheel;
    private Servo backwrist;
    private Servo frontwrist;
    private Servo colorLED;
    private DcMotor wheelMotor;
    NormalizedColorSensor colorSensor;

    // Private Members
    private boolean wristOut = false;
    private boolean slideIsOut = false;
    private final float[] hsvValues = new float[3];
    private Button wristInOut = new Button();
    private Button slideInOut = new Button();
    private Button runIntake = new Button();

    public IntakeSubsystem(LinearOpMode opMode){myOpMode = opMode;}

    public void initialize(boolean showTelemetry){
        leftLever = myOpMode.hardwareMap.get(Servo.class, "leftlever");
        rightLever = myOpMode.hardwareMap.get(Servo.class, "rightlever");
        leftWheel = myOpMode.hardwareMap.get(CRServo.class, "leftwheel");
        rightWheel = myOpMode.hardwareMap.get(CRServo.class, "rightwheel");
        wheelMotor = myOpMode.hardwareMap.get(DcMotor.class, "par");
        backwrist = myOpMode.hardwareMap.get(Servo.class, "frontwrist");
        frontwrist = myOpMode.hardwareMap.get(Servo.class, "backwrist");

        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "sample_color");
        colorLED = myOpMode.hardwareMap.get(Servo.class, "led");

        wheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor.setGain(8);

        slideIn();
        collectorOff();
        wristIn();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void update() {
        readSensors();
        runSlideControl();
        runStateMachine();
    }

    public void readSensors() {

        // process color/Range sensor
        double range = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        int hue = -1;
        if ((range > 0.5) && (range  < 6.5)) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            hue = (int)hsvValues[0];

            if (hue < 60) {
                gotSample = true;
                sampleColor = RED;
                colorLED.setPosition(.3);
            } else if (hue < 170) {
                gotSample = true;
                sampleColor = YELLOW;
                colorLED.setPosition(.35);
            }  else if (hue > 190) {
                gotSample = true;
                sampleColor = BLUE;
                colorLED.setPosition(.6);
            } else {
                gotSample = true;
                sampleColor = NONE;
                colorLED.setPosition(0);
            }
        } else {
            gotSample = false;
            sampleColor = NONE;
            colorLED.setPosition(0);
        }

        if (showTelemetry) {
            myOpMode.telemetry.addData("Sample", "%s h:%d %s Rng %.0f mm", gotSample ? "Full" : "Empty", hue , sampleColor, range);
        }
    }

    public void runStateMachine() {

        if (showTelemetry) {
            myOpMode.telemetry.addData("Intake State", "%S", currentState);
        }


        switch (currentState) {
            case INIT: {
                if (wristInOut.pressed(myOpMode.gamepad2.left_bumper)) {
                    wristOut();
                    setState(IntakeStates.HOME);
                } else {
                    slideIn();
                    wristIn();
                    collectorOff();
                }
                break;
            }

            case HOME:
                if (runIntake.pressed(myOpMode.gamepad2.dpad_down)) {
                    wristDown();
                    collectorIntake();
                    setState(IntakeStates.INTAKING);
                }  else if (myOpMode.gamepad2.dpad_up) {
                    collectorEject();
                } else {
                    collectorOff();
                }
                break;

            case INTAKING:
                if (runIntake.released(myOpMode.gamepad2.dpad_down)) {
                    wristOut();
                    collectorOff();
                    setState(IntakeStates.HOME);
                }
                else if (gotSample) {
                    wristOut();
                    collectorOff();
                    setState(IntakeStates.HOLDING_SAMPLE);
                }
                break;

            case HOLDING_SAMPLE:
                if (wristInOut.pressed(myOpMode.gamepad2.left_bumper)) {
                    wristIn();
                    // slideIn() // consider bringing both items in if clear of low rung.
                    setState(IntakeStates.RAISING_WRIST);
                } else if (runIntake.pressed(myOpMode.gamepad2.dpad_down)) {
                    wristDown();
                    collectorIntake();
                    setState(IntakeStates.INTAKING);
                }
                break;

            case RAISING_WRIST:
                if ((stateTime.time() > 1.0) && (!slideIsOut)) {
                    collectorIntake();
                    setState(IntakeStates.TRANSFERING_SAMPLE);
                }
                break;

            case TRANSFERING_SAMPLE:
                if (stateTime.time() > 1.0){
                    collectorOff();
                    setState(IntakeStates.HOME);
                }
                break;
        }
    }


    public void setState (IntakeStates newState){
        currentState = newState;
        stateTime.reset();
    }

    // ========================================================================================================

    public void runSlideControl() {
        if (slideInOut.pressed(myOpMode.gamepad2.right_bumper)){
            if (slideIsOut) {
                slideIn();
            } else {
                slideOut();
            }
        }
    }

    public void slideIn(){
        leftLever.setPosition(LEFT_LEVER_IN);
        rightLever.setPosition(RIGHT_LEVER_IN);
        slideIsOut = false;
    }

    public void slideOut(){
        leftLever.setPosition(LEFT_LEVER_OUT);
        rightLever.setPosition(RIGHT_LEVER_OUT);
        slideIsOut = true;
    }

    public void wristIn(){
        setWristPosition(WRIST_IN);
        wristOut = false;
    }

    public void wristOut(){
        setWristPosition(WRIST_OUT);
        wristOut = true;
    }

    public void wristDown(){
        setWristPosition(WRIST_DOWN);
        wristOut = true;
    }

    public void collectorIntake(){
        setCollector(INTAKE);
        if (wristOut){
            setWristPosition(WRIST_DOWN);
        }
    }

    public void collectorEject(){
        setCollector(EJECT);
    }

    public void collectorOff(){
        setCollector(OFF);
        if (wristOut){
            setWristPosition(WRIST_OUT);
        }
    }

    public void setWristPosition(double position){
        backwrist.setPosition(position);
        frontwrist.setPosition(position);
    }

    public void setCollector(double speed) {
        leftWheel.setPower(-speed);
        rightWheel.setPower(speed);
        wheelMotor.setPower(speed);
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

    public Action actionSetState(IntakeStates state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                setState(state);
                return false;
            }
        };
    }

    public Action actionWaitForState(IntakeStates state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                return currentState != state;
            }
        };
    }

}
