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
    private boolean      showTelemetry  = false;
    private ElapsedTime  stateTime      = new ElapsedTime();
    private double       outputPower    = 0;
    private IntakeStates currentState = IntakeStates.INIT;

    // Constants
    private final double INTAKE = 1;
    private final double EJECT = -1;
    private final double OFF = 0;
    private final double WRIST_IN = 0.42;
    private final double WRIST_OUT = 0.68;
    private final double WRIST_DOWN = 0.9;
    private final double SLIDE_TRANSIT_TIME = 1.0;
    private final double SLIDE_TRANSFER_TIME = 0.8;
    private final double SAMP_CHECKING_TIME = 0.11;

    private final double SAMP_NOT_COLLECTED_IN_TIME = 1;
    private final double SWEEP_TIMEOUT = 1.5;

    private final int    SLIDE_HOME = 0;
    private final int    ALMOST_HOME = 20;
    private final int    SLIDE_OUT  = 500;

    private final double HOME_POWER = -0.1;
    private final double HOLD_POWER = -0.1;
    private final int    HOME_MIN_MOVEMENT = 10;

    private final double GAIN = 0.01;
    private final double ACCEL_LIMIT = 4.0;
    private final double OUTPUT_LIMIT = 0.5;
    private final double TOLERANCE = 10.0;
    private final double DEADBAND = 5.0;


    // public members
    public boolean     gotSample   = false;
    public SampleColor sampleColor = NONE;
    public int         sampleHue = -1;

    //declaring servos for the intake
    private Servo backwrist;
    private Servo frontwrist;
    private Servo colorLED;
    private DcMotor leverMotor;
    private DcMotor wheelMotor;
    NormalizedColorSensor colorSensor;

    // Private Members
    private boolean wristOut = false;
    private boolean slideIsOut = false;
    private final float[] hsvValues = new float[3];

    private int leverSetPoint = 0;
    private int currentPosition = 0;
    private int lastPosition = 0;
    private boolean goingHome = false;

    // flags used in auto
    private boolean autoIntake = false;
    private boolean autoLower  = false;

    private Button wristInOut = new Button();
    private Button slideInOut = new Button();
    private Button runIntake = new Button();
    private ElapsedTime  slideTime = new ElapsedTime();
    private ProportionalControl positionControl = new ProportionalControl(GAIN, ACCEL_LIMIT, OUTPUT_LIMIT, TOLERANCE, DEADBAND, false);

    public IntakeSubsystem(LinearOpMode opMode){myOpMode = opMode;}

    public void initialize(boolean showTelemetry){
        wheelMotor = myOpMode.hardwareMap.get(DcMotor.class, "wheel");
        leverMotor = myOpMode.hardwareMap.get(DcMotor.class, "lever");
        leverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backwrist = myOpMode.hardwareMap.get(Servo.class, "frontwrist");
        frontwrist = myOpMode.hardwareMap.get(Servo.class, "backwrist");

        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "sample_color");
        colorLED = myOpMode.hardwareMap.get(Servo.class, "led");

        wheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor.setGain(8);

        if (!Globals.SLIDE_HOMED) {
            homeTheSlide();
        }

        slideIn();
        collectorOff();
        wristIn();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void update() {
        readSensors();
        runSlideControl();
        runControl();
        runStateMachine();

        if (showTelemetry) {
            myOpMode.telemetry.addData("INTAKE Got Color Hue", "%s %s %s %d", currentState, gotSample, sampleColor, sampleHue);
            myOpMode.telemetry.addData("SLIDE Pos SP Pwr", "%s %d %.1f %.2f", currentState, currentPosition, positionControl.getSetPoint(), outputPower);
            // myOpMode.telemetry.addData("Slide Pos", currentPosition);
            // myOpMode.telemetry.addData("Slide Pwr", outputPower * 1000);
            // myOpMode.telemetry.addData("Slide SP",  positionControl.getSetPoint());
        }
    }

    public void readSensors() {

        // process color/Range sensor
        double range = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        currentPosition = leverMotor.getCurrentPosition();

        sampleHue = -1;
        if ((range > 0.5) && (range  < 6.5)) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            sampleHue = (int)hsvValues[0];

            if (sampleHue < 68) {
                gotSample = true;
                sampleColor = RED;
                colorLED.setPosition(.3);
            } else if (sampleHue < 170) {
                gotSample = true;
                sampleColor = YELLOW;
                colorLED.setPosition(.35);
            }  else if (sampleHue > 190) {
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
    }

    public void runStateMachine() {
        switch (currentState) {
            case INIT: {
                if (wristInOut.pressed(myOpMode.gamepad2.left_bumper) || autoLower) {
                    autoLower = false;
                    wristOut();
                    setState(IntakeStates.HOME);
                } else if (runIntake.pressed(myOpMode.gamepad2.dpad_down) || autoIntake) {
                    wristDown();
                    collectorIntake();
                    setState(IntakeStates.INTAKING);
                } else {
                    slideIn();
                    wristIn();
                    collectorOff();
                }
                break;
            }

            case HOME:
                if (runIntake.pressed(myOpMode.gamepad2.dpad_down) || autoIntake) {
                    wristDown();
                    collectorIntake();
                    setState(IntakeStates.INTAKING);
                } else if (wristInOut.pressed(myOpMode.gamepad2.left_bumper)) {
                    wristIn();
                    if (gotSample) {
                        setState(IntakeStates.TILT_WRIST_IN);
                    } else {
                        slideIn();
                        setState(IntakeStates.INIT);
                    }
                }else if (myOpMode.gamepad2.dpad_up) {
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
                } else if (gotSample) {
                    setState(IntakeStates.CHECKING_SAMPLE);
                } else if (Globals.IS_AUTO && (stateTime.time() > SAMP_NOT_COLLECTED_IN_TIME)){
                    setState(IntakeStates.SWEEPING);
                }
                break;

            case SWEEPING:  // only used in auto
                if (gotSample) {
                    Globals.RC_SWEEP  = false;
                    Globals.RC_END    = true;
                    setState(IntakeStates.CHECKING_SAMPLE);
                } else if (stateTime.time() > SWEEP_TIMEOUT) {  // terminate sweep and move on.
                    collectorOff();
                    wristOut();
                    Globals.RC_SWEEP  = false;
                    Globals.RC_END    = true;
                    Globals.DID_NOT_SWEEP_SAMPLE = true; // Set flag to bypass dumping
                    setState(IntakeStates.GOT_SAMPLE);
                } else {
                    Globals.RC_SWEEP  = true; // Tell Drive Subsystem to Sweep back and forward.
                }
                break;

            case CHECKING_SAMPLE:
                if (stateTime.time() > SAMP_CHECKING_TIME) {
                    if (gotSample){
                        wristOut();
                        collectorOff();
                        setState(IntakeStates.GOT_SAMPLE);
                    } else {
                        setState(IntakeStates.INTAKING);
                    }
                }
                break;

            case GOT_SAMPLE:
                if (wristInOut.pressed(myOpMode.gamepad2.left_bumper) || autoIntake) {
                    wristIn();
                    autoIntake = false;
                    setState(IntakeStates.TILT_WRIST_IN);
                } else if (runIntake.pressed(myOpMode.gamepad2.dpad_down)) {
                    wristDown();
                    collectorIntake();
                    setState(IntakeStates.INTAKING);
                } else if (myOpMode.gamepad2.dpad_up) {
                    collectorEject();
                } else {
                    collectorOff();
                }
                break;

            case TILT_WRIST_IN:
                if ((stateTime.time() > 0.5) && (!slideIsOut && (slideTime.time() > SLIDE_TRANSIT_TIME))) {
                    collectorIntake();
                    setState(IntakeStates.TRANSFER);
                }
                break;

            case TRANSFER:
                if (stateTime.time() > SLIDE_TRANSFER_TIME){
                    collectorOff();
                    wristOut();
                    setState(IntakeStates.HOME);
                }
                break;
        }
    }

    public void setTargetPosition(int setPoint){
        leverSetPoint = setPoint;
        positionControl.reset(leverSetPoint);
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

    public void resetEncoders(){
        leverMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(10);
        leverMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void homeTheSlide(){
        goingHome = true;
        myOpMode.telemetry.addLine("Homing the slide");
        myOpMode.telemetry.update();
        leverMotor.setPower(HOME_POWER);
        myOpMode.sleep(100);
    }

    public void runControl(){
        outputPower = 0;

        if (goingHome) {
            // Decides if the slide is still in the homing motion or if it has stopped and is homed
            if(Math.abs(currentPosition-lastPosition) < HOME_MIN_MOVEMENT){
                outputPower = 0;
                resetEncoders();
                goingHome = false;
                Globals.SLIDE_HOMED = true;
            } else {
                outputPower = HOME_POWER;
            }
            lastPosition = currentPosition;
            myOpMode.sleep(50);
        } else {
            outputPower = positionControl.getOutput(currentPosition);
            if ((positionControl.getSetPoint() == SLIDE_HOME ) && (currentPosition < ALMOST_HOME)) {
                outputPower = HOLD_POWER;
            }
        }
        leverMotor.setPower(outputPower);
    }


    public void slideIn(){
        // start time when we start bringing slides in so we can allow enough time for it to retract.
        if (slideIsOut) {
            slideTime.reset();
        }
        setTargetPosition(SLIDE_HOME);
        slideIsOut = false;
    }

    public void slideOut(){
        setTargetPosition(SLIDE_OUT);
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
                return (currentState != state);
            }

        };
    }

    public Action actionIntakeIt(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                autoIntake = true;
                return false;
            }
        };
    }

    public Action actionLowerIt(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                autoLower = true;
                return false;
            }
        };
    }
}
