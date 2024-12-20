package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.subsystems.ArmStates.CANCEL;
import static org.firstinspires.ftc.teamcode.subsystems.ArmStates.CLIPPING;
import static org.firstinspires.ftc.teamcode.subsystems.ArmStates.GRABBED;
import static org.firstinspires.ftc.teamcode.subsystems.ArmStates.GRABBING;
import static org.firstinspires.ftc.teamcode.subsystems.ArmStates.LIFTING;
import static org.firstinspires.ftc.teamcode.subsystems.ArmStates.LOWERING;
import static org.firstinspires.ftc.teamcode.subsystems.ArmStates.READY;
import static org.firstinspires.ftc.teamcode.subsystems.ArmStates.READY_CLIP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ArmSubsystem {

    // Standard SubSystem Members:
    private LinearOpMode myOpMode;
    private boolean     showTelemetry   = false;
    private ArmStates   currentState    = READY;
    private double      outputPower     = 0;
    private ElapsedTime stateTime       = new ElapsedTime();

    // Constants
    private final double CLAW_OPEN = 0.3;
    private final double CLAW_CLOSED = 0.55;
    private final double LOOSE_GRIP = 0.5;
    private final double HOME_POWER = -0.2;
    private final int    HOME_MIN_MOVEMENT = 10;

    private final double GAIN = 0.0025;
    private final double ACCEL_LIMIT = 10.0;
    private final double OUTPUT_LIMIT = 1.0;
    private final double TOLERANCE = 30.0;
    private final double DEADBAND = 20.0;

    private final int CLIPPING_POSITION = 1000;
    private final int CLIPPED_POSITON = 700;
    private final int HOME_POSITION = 0;

    private DcMotor arm;      // motor used to control the arm
    private Servo claw;       // Specimen Claw

    // Private Members
    private int armSetPoint = 0;
    private int currentPosition = 0;
    private int lastPosition = 0;
    private boolean timeToClip = false;
    private boolean goingHome = false;

    private Button grabScore = new Button();
    private ProportionalControl positionControl = new ProportionalControl(GAIN, ACCEL_LIMIT, OUTPUT_LIMIT, TOLERANCE, DEADBAND, false);

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

        if (!Globals.ARM_HOMED) {
            homeTheArm();
        }

        setTargetPosition(currentPosition);

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void update() {
        readSensors();
        runControl();
        runStateMachine();

        if (showTelemetry) {
            myOpMode.telemetry.addData("ARM Pos SP Pwr", "%s %d %.1f %.2f", currentState, currentPosition, positionControl.getSetPoint(), outputPower);
            //  myOpMode.telemetry.addData("Arm Pos", currentPosition);
            //  myOpMode.telemetry.addData("Arm Pwr", outputPower * 1000);
            //  myOpMode.telemetry.addData("Arm SP",  positionControl.getSetPoint());
        }
    }

    public void readSensors(){
        currentPosition = arm.getCurrentPosition();
    }
    public boolean inPosition (){
        return positionControl.inPosition();
    }

    public void runStateMachine (){
        switch (currentState){

            case READY:{
                if (grabScore.pressed(myOpMode.gamepad1.right_bumper)){
                    claw.setPosition(CLAW_CLOSED);
                    setState(GRABBING);
                } else {
                    //stop();
                }
                break;
            }

            case GRABBING:{
                claw.setPosition(CLAW_CLOSED);
                if (stateTime.time() > 0.3){
                    setState(GRABBED);
                }
                break;
            }

            case GRABBED:{
                setTargetPosition(CLIPPING_POSITION);
                setState(LIFTING);
                break;
            }


            case LIFTING:{
                if(positionControl.inPosition()){
                    setState(READY_CLIP);
                }
                break;
            }

            case READY_CLIP:{
                if(grabScore.pressed(myOpMode.gamepad1.right_bumper) || timeToClip){
                    setTargetPosition(CLIPPED_POSITON);
                    // claw.setPosition(LOOSE_GRIP);
                    timeToClip = false;
                    setState(CLIPPING);
                } else if (myOpMode.gamepad1.right_trigger > 0.25){
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
                if(stateTime.time() > 0.4){
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
        goingHome = true;
        myOpMode.telemetry.addLine("Homing the arm");
        myOpMode.telemetry.update();
        arm.setPower(HOME_POWER);
        myOpMode.sleep(100);
    }

    public void resetEncoders(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(10);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runControl(){
        outputPower = 0;

        if (goingHome) {
            // Decides if the lift is still in the homing motion or if it has stopped and is homed
            int position = arm.getCurrentPosition();
            if(Math.abs(position-lastPosition) < HOME_MIN_MOVEMENT){
                outputPower = 0;
                resetEncoders();
                goingHome = false;
                Globals.ARM_HOMED = true;
                positionControl.reset(HOME_POSITION);
                setState(READY);
            } else {
                outputPower = HOME_POWER;
            }
            lastPosition = position;
            myOpMode.sleep(50);

        } else {
            outputPower = positionControl.getOutput(currentPosition);
        }

        arm.setPower(outputPower);
    }

    public void setTargetPosition(int setPoint){
        armSetPoint = setPoint;
        positionControl.reset(armSetPoint);
    }

    public void openClaw (){
        claw.setPosition(CLAW_OPEN);
    }
    public void closeClaw(){
        claw.setPosition(CLAW_CLOSED);
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

    public Action actionClipIt(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                timeToClip = true;
                return false;
            }
        };
    }

    public Action actionSetState(ArmStates state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                setState(state);
                return false;
            }
        };
    }

    public Action actionWaitForState(ArmStates state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                return (currentState != state);
            }
        };
    }

    public Action actionClaw(boolean openClaw){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
               if(openClaw){
                   claw.setPosition(CLAW_OPEN);
               } else {
                   claw.setPosition(CLAW_CLOSED);
               }
                return false;
            }
        };
    }

    public Action actionUpdateTelemetry(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                myOpMode.telemetry.update();
                return true;
            }
        };
    }

}