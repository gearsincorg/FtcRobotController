package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ArmStates.LIFTED;
import static org.firstinspires.ftc.teamcode.ArmStates.LIFTING;
import static org.firstinspires.ftc.teamcode.ArmStates.LOWERING;
import static org.firstinspires.ftc.teamcode.ArmStates.READY_OPEN;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystem {

    private final double SLOPE = 0.0123;
    private final double OFFSET = 9.5;
    private final double HOLD_POWER = 0.1;
    private final double LIFTING_POWER = 0.5;
    private final double LOWERING_POWER = -0.6;

    private DcMotor arm;      //motor used to control the arm
    DigitalChannel upSensor;
    DigitalChannel downSensor;

    private LinearOpMode myOpMode;
    private boolean showTelemetry     = false;
    private int currentPosition = 0;
    private int lastPosition = 0;
    private boolean isDown = false;
    private boolean isUp = false;
    private ArmStates currentState = READY_OPEN;
    private ElapsedTime stateTime = new ElapsedTime();

    public ArmSubsystem(LinearOpMode opMode){
        myOpMode = opMode;
    }

    public void initialize(boolean showTelemetry){
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Requires motor encoder cables to be hooked up.

        upSensor = myOpMode.hardwareMap.get(DigitalChannel.class, "upsensor");
        downSensor = myOpMode.hardwareMap.get(DigitalChannel.class, "downsensor");

        upSensor.setMode(DigitalChannel.Mode.INPUT);
        downSensor.setMode(DigitalChannel.Mode.INPUT);
        myOpMode.telemetry.addData("DigitalTouchSensorExample", "Press start to continue...");
        myOpMode.telemetry.addData("DigitalTouchSensorExample", "Press start to continue...");
        myOpMode.telemetry.update();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void readSensors(){
        currentPosition = arm.getCurrentPosition();

        isUp = !upSensor.getState();
        isDown = !downSensor.getState();

        if (showTelemetry) {
            myOpMode.telemetry.addData("arm encoder", "%d", currentPosition);
            myOpMode.telemetry.addData("is up", isUp);
            myOpMode.telemetry.addData("is down", isDown);
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

            case READY_OPEN: {
                if(myOpMode.gamepad1.triangle){
                    setState(LIFTING);
                } else {
                    stop();
                }
                break;
            }

            case LIFTING: {
                if(isUp == true){
                    setState(LIFTED);
                } else {
                   arm.setPower(LIFTING_POWER);
                }
                break;
            }

            case LIFTED: {
                if(myOpMode.gamepad1.cross || !isUp){
                    setState(LOWERING);
                } else {
                    stop();
                }
                break;
            }

            case LOWERING: {
                if(isDown){
                    setState(READY_OPEN);
                } else {
                    arm.setPower(LOWERING_POWER);
                }
            }
        }
    }

    public void setState (ArmStates newState){
        currentState = newState;
        stateTime.reset();

    }



}
