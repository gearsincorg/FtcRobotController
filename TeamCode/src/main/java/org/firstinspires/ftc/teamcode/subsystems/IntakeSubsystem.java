package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.SampleColor.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.SampleColor.NONE;
import static org.firstinspires.ftc.teamcode.subsystems.SampleColor.RED;
import static org.firstinspires.ftc.teamcode.subsystems.SampleColor.YELLOW;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {

    // public members
    public boolean     gotSample   = false;
    public SampleColor sampleColor = NONE;

    //declaring telemetry
    private boolean showTelemetry     = false;

    //functions for the servos
    private final double LEFT_LEVER_IN = 0.53;
    private final double RIGHT_LEVER_IN = 0.47;
    private final double LEFT_LEVER_OUT = .4;
    private final double RIGHT_LEVER_OUT = .60;
    private final double INTAKE = 1;
    private final double EJECT = -1;
    private final double OFF = 0;
    private final double WRIST_IN = 0.4;
    private final double WRIST_OUT = 0.68;
    private final double WRIST_COLLECT = 0.87;


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


    private boolean wristOut = false;
    private final float[] hsvValues = new float[3];

    private LinearOpMode myOpMode;

    public IntakeSubsystem(LinearOpMode opMode){myOpMode = opMode;}

    public void initialize(boolean showTelemetry){
        leftLever = myOpMode.hardwareMap.get(Servo.class, "leftlever");
        rightLever = myOpMode.hardwareMap.get(Servo.class, "rightlever");
        leftWheel = myOpMode.hardwareMap.get(CRServo.class, "leftwheel");
        rightWheel = myOpMode.hardwareMap.get(CRServo.class, "rightwheel");
        wheelMotor = myOpMode.hardwareMap.get(DcMotor.class, "par");
        backwrist = myOpMode.hardwareMap.get(Servo.class, "frontwrist");
        frontwrist = myOpMode.hardwareMap.get(Servo.class, "backwrist");

        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorLED = myOpMode.hardwareMap.get(Servo.class, "led");

        wheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor.setGain(8);

        leverIn();
        off();
        wristIn();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void update() {

        // process color sensor
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
            myOpMode.telemetry.addData("Sample", "hue %d %s Color %s range %f",hue ,  gotSample ? "Found" : "Not Found", sampleColor, range);
        }
    }

    public void leverOut(){
        leftLever.setPosition(LEFT_LEVER_OUT);
        rightLever.setPosition(RIGHT_LEVER_OUT);
    }

    public void leverIn(){
        leftLever.setPosition(LEFT_LEVER_IN);
        rightLever.setPosition(RIGHT_LEVER_IN);
    }

    public void wristIn(){
        setWrist(WRIST_IN);
        wristOut = false;
    }

    public void wristOut(){
       setWrist(WRIST_OUT);
       wristOut = true;
    }

    public void intake(){
        leftWheel.setPower(-INTAKE);
        rightWheel.setPower(INTAKE);
        wheelMotor.setPower(INTAKE);
        if (wristOut){
            setWrist(WRIST_COLLECT);
        }


    }

    public void eject(){
        leftWheel.setPower(-EJECT);
        rightWheel.setPower(EJECT);
        wheelMotor.setPower(EJECT);
    }

    public void off(){
        leftWheel.setPower(OFF);
        rightWheel.setPower(OFF);
        wheelMotor.setPower(OFF);
        if (wristOut){
            setWrist(WRIST_OUT);
        }
    }

    public void setWrist(double position){
        backwrist.setPosition(position);
        frontwrist.setPosition(position);
    }

}
