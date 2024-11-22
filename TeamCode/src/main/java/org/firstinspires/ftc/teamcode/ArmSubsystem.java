package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.ArmStates.CANCEL;
import static org.firstinspires.ftc.teamcode.ArmStates.CLIPPING;
import static org.firstinspires.ftc.teamcode.ArmStates.GRABBING;
import static org.firstinspires.ftc.teamcode.ArmStates.LIFTING;
import static org.firstinspires.ftc.teamcode.ArmStates.LOWERING;
import static org.firstinspires.ftc.teamcode.ArmStates.READY;
import static org.firstinspires.ftc.teamcode.ArmStates.READY_TO_CLIP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystem {

    private final double SLOPE = 0.0123;
    private final double OFFSET = 9.5;
    private final double HOLD_POWER = 0.1;
    private final double LIFTING_POWER = 0.5;
    private final double LOWERING_POWER = -0.6;
    private final double CLAW_OPEN = 0.3;
    private final double CLAW_CLOSED = 0.55;
    private final double LOOSE_GRIP = 0.5;
    private final double HOME_POWER = -0.15;
    private final int HOME_MIN_MOVEMENT = 10;

    private final double GAIN = 1.0 / 200.0;
    private final double ACCEL_LIMIT = 7.0;
    private final double OUTPUT_LIMIT = 0.75;
    private final double TOLERANCE = 20.0;
    private final double DEADBAND = 10.0;

    private final int CLIPPING_POSITION = 950;
    private final int CLIPPED_POSITON = 600;
    private final int HOME_POSITION = 0;

    private DcMotor arm;      //motor used to control the arm
    private Servo claw;

    private LinearOpMode myOpMode;
    private boolean showTelemetry     = false;
    private int armSetPoint = 0;
    private int currentPosition = 0;
    private int lastPosition = 0;
    private ArmStates currentState = READY;
    private ElapsedTime stateTime = new ElapsedTime();
    private ProportionalControl positionControl = new ProportionalControl(GAIN, ACCEL_LIMIT, OUTPUT_LIMIT, TOLERANCE, DEADBAND, false);
    private boolean timeToClip = false;

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

        homeTheArm();
        setTargetPosition(currentPosition);

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void update() {
        readSensors();
        runControl();
        runStateMachine();
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

    public void clipIt(){
        timeToClip = true;
    }

    public boolean isHome(){
        return currentState == READY;
    }

    /**
     * closes the claw to grab specimen
     */
    public void autoGrab() {
        claw.setPosition(CLAW_CLOSED);
    }

    public void autoGoToBackPosition(){
        setTargetPosition(CLIPPING_POSITION);
        setState(LIFTING);
    }

    public boolean inPosition (){
        return positionControl.inPosition;
    }

    public void runStateMachine (){

        if (showTelemetry) {
            myOpMode.telemetry.addData("Arm State", "%S", currentState);
        }

        switch (currentState){

            case READY:{
                if (myOpMode.gamepad1.right_bumper){
                    autoGrab();
                    setState(GRABBING);
                } else {
                    stop();
                }
                break;
            }

            case GRABBING:{
                if (stateTime.time() > 0.2){
                    setTargetPosition(CLIPPING_POSITION);
                    setState(LIFTING);
                }
                break;
            }

            case LIFTING:{
                if(positionControl.inPosition()){
                    setState(READY_TO_CLIP);
                }
                break;
            }

            case READY_TO_CLIP:{
                if(myOpMode.gamepad1.square || timeToClip){
                    setTargetPosition(CLIPPED_POSITON);
                    claw.setPosition(LOOSE_GRIP);
                    timeToClip = false;
                    setState(CLIPPING);
                } else if (myOpMode.gamepad1.circle){
                    setTargetPosition(HOME_POSITION);
                    claw.setPosition(CLAW_OPEN);
                    setState(CANCEL);
                }
                break;
            }

            case CANCEL:{
                if (positionControl.inPosition()){
                    setState(READY);
                }
                break;
            }

            case CLIPPING:{
                if(stateTime.time() > 0.5){
                    setTargetPosition(HOME_POSITION);
                    claw.setPosition(CLAW_OPEN);
                    setState(LOWERING);
                }
                break;
            }

            case LOWERING:{
                if(positionControl.inPosition()){
                    setState(READY);
                }
            }
        }
    }



    public void setState (ArmStates newState){
        currentState = newState;
        stateTime.reset();

    }

    public void homeTheArm(){
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
        readSensors();
    }

    public void runControl(){
        double motorPower = positionControl.getOutput(currentPosition);
        arm.setPower(motorPower);
        if (showTelemetry) {
            myOpMode.telemetry.addData("arm power", motorPower);
        }
    }

    public void setTargetPosition(int setPoint){
        armSetPoint = setPoint;
        positionControl.reset(armSetPoint);
    }

    //-------------------------------------------------------------------------
    //ACTION  CLASSES
    //-------------------------------------------------------------------------
    public class ActionUpdate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            update();
            return true;
        }
    }

    public Action actionUpdate(){
        return new ActionUpdate();
    }

    public class ActionClipIt implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            clipIt();
            return false;
        }
    }

    public Action actionClipIt(){
        return new ActionClipIt();
    }

    public class ActionWaitForHome implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            return currentState != READY;
        }
    }

    public Action actionWaitForHome(){
        return new ActionWaitForHome();
    }
}