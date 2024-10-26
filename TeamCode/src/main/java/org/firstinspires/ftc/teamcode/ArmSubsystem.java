package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.ArmStates.READY;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystem {

    private final double SLOPE = 0.0123;
    private final double OFFSET = 9.5;
    private final double HOLD_POWER = 0.1;
    private final double LIFTING_POWER = 0.5;
    private final double LOWERING_POWER = -0.6;
    private final double CLAW_OPEN = 0;
    private final double CLAW_CLOSED = 1;
    private final double HOME_POWER = -0.2;
    private final int HOME_MIN_MOVEMENT = 10;

    private DcMotor arm;      //motor used to control the arm
    DigitalChannel upSensor;
    DigitalChannel downSensor;
    private Servo claw;

    private LinearOpMode myOpMode;
    private boolean showTelemetry     = false;
    private int armSetPoint = 0;
    private int currentPosition = 0;
    private int lastPosition = 0;
    private ArmStates currentState = READY;
    private ElapsedTime stateTime = new ElapsedTime();

    public ArmSubsystem(LinearOpMode opMode){
        myOpMode = opMode;
    }

    public void initialize(boolean showTelemetry){
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.

        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_OPEN);

        HomeTheArm();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void readSensors(){
        currentPosition = arm.getCurrentPosition();

        if (showTelemetry) {
            myOpMode.telemetry.addData("arm encoder", "%d", currentPosition);
        }
    }

    public void setPower(double power){
        arm.setPower(power);
    }

    public void stop(){
        arm.setPower(0);
    }

    public void hold(){
        arm.setPower(HOLD_POWER);
    }

    public void runStateMachine (){

        readSensors();

        if (showTelemetry) {
            myOpMode.telemetry.addData("Arm State", "%S", currentState);
        }

        switch (currentState){


        }
    }

    public void setState (ArmStates newState){
        currentState = newState;
        stateTime.reset();

    }

    public void HomeTheArm (){
        arm.setPower(HOME_POWER);
        lastPosition = arm.getCurrentPosition();
        myOpMode.sleep(250);
        while (myOpMode.opModeInInit()) {
            readSensors();
            int movement = Math.abs(currentPosition - lastPosition);
            if (movement <= HOME_MIN_MOVEMENT){
                stop();
                myOpMode.sleep(500);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            } else {
                lastPosition = currentPosition;
                myOpMode.sleep(50);
            }

        }
        stop();
    }


}
