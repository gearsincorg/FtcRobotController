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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {

    private final boolean IMMEDIATE_TRANSFER = true;

    // Standard SubSystem Members:
    private LinearOpMode myOpMode;
    private boolean      showTelemetry  = false;
    private ElapsedTime  stateTime      = new ElapsedTime();
    private double       outputPower    = 0;
    private IntakeStates currentState = IntakeStates.INIT;

    // Constants
    private final double INTAKE = 1;
    private final double TRANSFER = 0.8;
    private final double EJECT = -1;

    private final double OFF = 0;
    private final double WRIST_IN = 0.43;
    private final double WRIST_OUT = 0.67;
    private final double WRIST_DOWN = 0.8;
    private final double SLIDE_TRANSIT_TIME = 1.0;
    private final double SLIDE_TRANSFER_TIME = 1.0;
    private final double SERVO_TILT_TIME = 0.65;
    private final double UNJAM_IN = 0.4;
    private final double UNJAM_OUT = -0.4;

    private final double SAMP_NOT_COLLECTED_IN_TIME = 1.25;
    private final double SWEEP_TIMEOUT = 2;
    private final double EJECT_TIMEOUT = 0.5;

    private final int    SLIDE_HOME = 0;
    private final int    ALMOST_HOME = 20;
    private final int    SLIDE_OUT  = 500;

    private final double HOME_POWER = -0.25;
    private final double HOLD_POWER = -0.15;
    private final int    HOME_MIN_MOVEMENT = 10;

    private final double GAIN = 0.01;
    private final double ACCEL_LIMIT = 4.0;
    private final double OUTPUT_LIMIT = 0.5;
    private final double TOLERANCE = 10.0;
    private final double DEADBAND = 5.0;

    private final double LED_RED_VALUE = 0.3;
    private final double LED_YELLOW_VALUE = 0.35;
    private final double LED_GREEN_VALUE = 0.5;
    private final double LED_BLUE_VALUE = 0.6;
    private final double LED_OFF_VALUE = 0.0;

    // public members
    public boolean     gotSample   = false;
    public boolean     gotWrongSample   = false;
    public SampleColor sampleColor = NONE;
    public int         sampleHue   = -1;
    public int         sampleRange = 100;

    //declaring servos for the intake
    private Servo backwrist;
    private Servo frontwrist;
    private Servo colorLED;
    private DcMotorEx leverMotor;
    private DcMotorEx wheelMotor;
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
    private boolean autoTranser = false;
    private boolean autoLower  = false;

    private Button wristInOut = new Button();
    private Button slideInOut = new Button();
    private Button runIntake = new Button();
    private ElapsedTime  slideTime = new ElapsedTime();
    private ProportionalControl positionControl = new ProportionalControl(GAIN, ACCEL_LIMIT, OUTPUT_LIMIT, TOLERANCE, DEADBAND, false);

    public IntakeSubsystem(LinearOpMode opMode){myOpMode = opMode;}

    public void initialize(boolean showTelemetry){
        wheelMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "wheel");
        wheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leverMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "lever");
        leverMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        backwrist = myOpMode.hardwareMap.get(Servo.class, "frontwrist");
        frontwrist = myOpMode.hardwareMap.get(Servo.class, "backwrist");

        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "sample_color");
        colorLED = myOpMode.hardwareMap.get(Servo.class, "led");

        wheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        colorSensor.setGain(8);

        // Always home in Auto, and sometimes in teleop, if running from a fresh restart
        if (Globals.IS_AUTO || !Globals.SLIDE_HOMED) {
            homeTheSlide();
        }

        slideIn();
        collectorOff();
        wristIn();
        Globals.RC_END = false;
        Globals.DID_NOT_SWEEP_SAMPLE = false;

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void update() {
        readSensors();
        runSlideControl();
        runControl();
        runStateMachine();

        if (showTelemetry) {
            myOpMode.telemetry.addData("INTAKE Got Color Hue Rng", "%s %s %s %d %d", currentState, gotSample, sampleColor, sampleHue, sampleRange);
            myOpMode.telemetry.addData("SLIDE Pos SP Pwr", "%s %d %d %.2f", currentState, currentPosition, (int)positionControl.getSetPoint(), outputPower);
            // myOpMode.telemetry.addData("Slide Pos", currentPosition);
            // myOpMode.telemetry.addData("Slide Pwr", outputPower * 1000);
            // myOpMode.telemetry.addData("Slide SP",  positionControl.getSetPoint());
        }
        myOpMode.telemetry.addData("GLOBALS", "%s %s", Globals.RC_SWEEP ? "SWEEP" : "NoSWEEP", Globals.RC_END ? "END" : "RUN");
    }

    public void readSensors() {

        // process color/Range sensor
        sampleRange = (int)(((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM));

        currentPosition = leverMotor.getCurrentPosition();

        sampleHue = -1;
        if ((sampleRange > 1) && (sampleRange < 24)) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            sampleHue = (int)hsvValues[0];

            if (sampleHue < 68) {
                gotSample = true;
                sampleColor = RED;
                gotWrongSample = (Globals.ALLIANCE_COLOR == AllianceColor.BLUE);
                colorLED.setPosition(LED_RED_VALUE);
            } else if (sampleHue < 170) {
                gotSample = true;
                sampleColor = YELLOW;
                gotWrongSample = false;
                colorLED.setPosition(LED_YELLOW_VALUE);
            }  else if (sampleHue > 190) {
                gotSample = true;
                sampleColor = BLUE;
                gotWrongSample = (Globals.ALLIANCE_COLOR == AllianceColor.RED);
                colorLED.setPosition(LED_BLUE_VALUE);
            } else {
                gotSample = true;
                gotWrongSample = false;
                sampleColor = NONE;
                colorLED.setPosition(LED_OFF_VALUE);
            }
        } else {
            gotSample = false;
            sampleColor = NONE;
            if (!myOpMode.opModeInInit()) {
                colorLED.setPosition(LED_OFF_VALUE);
            }
        }
    }

    public void runStateMachine() {
        switch (currentState) {
            case INIT: {
                if (autoLower) {
                    autoLower = false;
                    wristDown();
                    collectorIntake();
                    setState(IntakeStates.INTAKING);
                } else if (wristInOut.pressed(myOpMode.gamepad2.left_bumper)) {
                    wristOut();
                    setState(IntakeStates.HOME);
                } else if (runIntake.pressed(myOpMode.gamepad2.dpad_down) || autoTranser) {
                    wristDown();
                    collectorIntake();
                    setState(IntakeStates.INTAKING);
                } else if (myOpMode.gamepad2.right_bumper) {
                    slideOut();
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
                if (autoLower){
                    autoLower = false;
                    wristDown();
                    collectorIntake();
                    setState(IntakeStates.INTAKING);
                } else if (runIntake.pressed(myOpMode.gamepad2.dpad_down) || autoTranser) {
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
                    Globals.RC_SWEEP   = false; // disable the sweep action
                    collectorOff();
                    wristIn();
                    setState(IntakeStates.GOT_SAMPLE);
                } else if (Globals.IS_AUTO && (stateTime.time() > SAMP_NOT_COLLECTED_IN_TIME)){
                    Globals.RC_SWEEP  = true; // Tell Drive Subsystem to Sweep back and forward.
                    setState(IntakeStates.SWEEPING);
                }
                break;

            case SWEEPING:  // only used in auto
                if (gotSample) {
                    Globals.RC_SWEEP   = false; // disable the sweep action
                    collectorOff();
                    wristIn();
                    setState(IntakeStates.GOT_SAMPLE);
                } else if (stateTime.time() > SWEEP_TIMEOUT) {  // terminate sweep and move on.
                    collectorOff();
                    wristIn();
                    Globals.DID_NOT_SWEEP_SAMPLE = true; // Set flag to bypass dumping
                    setState(IntakeStates.GOT_SAMPLE);
                } else {
                    Globals.RC_SWEEP  = true; // Keep Sweeping back and forward.
                }
                break;

            case GOT_SAMPLE:
                Globals.RC_END    = true;  // disable the sweep action
                if (gotWrongSample) {
                    collectorEject();
                    setState(IntakeStates.EJECTING_SAMPLE);
                } else if (wristInOut.pressed(myOpMode.gamepad2.left_bumper) || autoTranser || IMMEDIATE_TRANSFER) {
                    autoTranser = false;
                    slideIn();  /// NEW !!!!!!
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

            case EJECTING_SAMPLE:
                if (stateTime.time() > EJECT_TIMEOUT) {  // Run eject for short time then go back to intake
                    wristDown();
                    collectorIntake();
                    setLEDoff();
                    setState(IntakeStates.INTAKING);
                }
                break;

            case TILT_WRIST_IN:
                if (myOpMode.gamepad2.dpad_up) {
                    collectorEject();
                    setState(IntakeStates.HOME);
                } else if ((stateTime.time() > SERVO_TILT_TIME) && (!slideIsOut && (slideTime.time() > SLIDE_TRANSIT_TIME))) {
                    collectorTransfer();
                    setState(IntakeStates.TRANSFER);
                } else {
                    if (gotSample){
                        // check to see if sample turned up late
                        Globals.DID_NOT_SWEEP_SAMPLE = false;
                        setCollector(UNJAM_OUT);
                    } else {
                        setCollector(UNJAM_IN);
                    }
                }
                break;

            case TRANSFER:
                setLEDoff();
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

    public void setLEDtoAllianceColor() {
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED) {
            colorLED.setPosition(LED_RED_VALUE);
        } else {
            colorLED.setPosition(LED_BLUE_VALUE);
        }
    }

    public void setLEDoff() {
        colorLED.setPosition(LED_OFF_VALUE);
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
        leverMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(10);
        leverMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
            myOpMode.telemetry.addLine("Homing the slide now");
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
        if (wristOut){
            setWristPosition(WRIST_DOWN);
        }
        setCollector(INTAKE);
    }

    public void collectorTransfer(){
        setCollector(TRANSFER);
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
                autoTranser = true;
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
