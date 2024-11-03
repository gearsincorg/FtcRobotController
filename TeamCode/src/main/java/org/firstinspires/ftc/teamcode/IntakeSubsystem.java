package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    //declaring telemetry
    private boolean showTelemetry     = false;

    //functions for the servos
    private final double LEFT_LEVER_IN = 0.5;
    private final double RIGHT_LEVER_IN = 0.5;
    private final double LEFT_LEVER_OUT = 1;
    private final double RIGHT_LEVER_OUT = 0;
    private final double TILT_UP = 0.5;
    private final double TILT_DOWN = 0;
    private final double INTAKE = -1;
    private final double EJECT = 1;
    private final double OFF = 0;

    //declaring servos for the intake
    private Servo leftLever;
    private Servo rightLever;
    private CRServo leftWheel;
    private CRServo rightWheel;
    private Servo tilt;

    private LinearOpMode myOpMode;

    public IntakeSubsystem(LinearOpMode opMode){myOpMode = opMode;}

    public void initialize(boolean showTelemetry){
        leftLever = myOpMode.hardwareMap.get(Servo.class, "leftlever");
        rightLever = myOpMode.hardwareMap.get(Servo.class, "rightlever");
        leftWheel = myOpMode.hardwareMap.get(CRServo.class, "leftwheel");
        rightWheel = myOpMode.hardwareMap.get(CRServo.class, "rightwheel");
        tilt = myOpMode.hardwareMap.get(Servo.class, "tilt");

        in();
        up();
        off();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void out(){
        leftLever.setPosition(LEFT_LEVER_OUT);
        rightLever.setPosition(RIGHT_LEVER_OUT);
    }

    public void in(){
        leftLever.setPosition(LEFT_LEVER_IN);
        rightLever.setPosition(RIGHT_LEVER_IN);
    }

    public void up(){
        tilt.setPosition(TILT_UP);
    }

    public void down(){
        tilt.setPosition(TILT_DOWN);
    }

    public void intake(){
        leftWheel.setPower(INTAKE);
        rightWheel.setPower(INTAKE);
    }

    public void eject(){
        leftWheel.setPower(EJECT);
        rightWheel.setPower(EJECT);
    }

    public void off(){
        leftWheel.setPower(OFF);
        rightWheel.setPower(OFF);
    }
}
