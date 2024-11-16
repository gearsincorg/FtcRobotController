package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    //declaring telemetry
    private boolean showTelemetry     = false;

    //functions for the servos
    private final double LEFT_LEVER_IN = 0.53;
    private final double RIGHT_LEVER_IN = 0.47;
    private final double LEFT_LEVER_OUT = .4;
    private final double RIGHT_LEVER_OUT = .60;
    private final double INTAKE = -1;
    private final double EJECT = 1;
    private final double OFF = 0;
    private final double WRIST_IN = 0;
    private final double WRIST_OUT = 1;

    //declaring servos for the intake
    private Servo leftLever;
    private Servo rightLever;
    private CRServo leftWheel;
    private CRServo rightWheel;
    private Servo backwrist;
    private Servo frontwrist;

    private LinearOpMode myOpMode;

    public IntakeSubsystem(LinearOpMode opMode){myOpMode = opMode;}

    public void initialize(boolean showTelemetry){
        leftLever = myOpMode.hardwareMap.get(Servo.class, "leftlever");
        rightLever = myOpMode.hardwareMap.get(Servo.class, "rightlever");
        leftWheel = myOpMode.hardwareMap.get(CRServo.class, "leftwheel");
        rightWheel = myOpMode.hardwareMap.get(CRServo.class, "rightwheel");
        backwrist = myOpMode.hardwareMap.get(Servo.class, "frontwrist");
        frontwrist = myOpMode.hardwareMap.get(Servo.class, "backwrist");

        in();
        off();
        wristIn();

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

    public void wristIn(){
        backwrist.setPosition(WRIST_IN);
        frontwrist.setPosition(WRIST_IN);
    }

    public void wristOut(){
        backwrist.setPosition(WRIST_OUT);
        frontwrist.setPosition(WRIST_OUT);
    }

    public void intake(){
        leftWheel.setPower(-INTAKE);
        rightWheel.setPower(INTAKE);
    }

    public void eject(){
        leftWheel.setPower(-EJECT);
        rightWheel.setPower(EJECT);
    }

    public void off(){
        leftWheel.setPower(OFF);
        rightWheel.setPower(OFF);
    }
}
