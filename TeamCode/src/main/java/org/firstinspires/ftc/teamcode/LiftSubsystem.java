package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ArmStates.READY;
import static org.firstinspires.ftc.teamcode.LiftStates.DUMPED;
import static org.firstinspires.ftc.teamcode.LiftStates.HOME;
import static org.firstinspires.ftc.teamcode.LiftStates.LIFTING;
import static org.firstinspires.ftc.teamcode.LiftStates.LOWERING;
import static org.firstinspires.ftc.teamcode.LiftStates.READY_TO_SCORE;
import static org.firstinspires.ftc.teamcode.LiftStates.SAMPLE_HELD;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftSubsystem {

    public final double MAX_HEIGHT = 45;
    public final double MIN_HEIGHT = 8.5;
    public final double HIGH_BASKET = 40;
    public final double LOW_BASKET = 25 ;
    private final double HOLD_POWER = 0.05;
    private final double HOME_POWER = -0.6;
    private final double PITCH = 0.5;
    private final double YAW = 0.5;

    private final double GAIN = 1.0;
    private final double ACCEL_LIMIT = 8.0;
    private final double OUTPUT_LIMIT = 1;
    private final double TOLERANCE = 0.75;
    private final double DEADBAND = 0.25;

    private final double SLOPE = 0.0123;
    private final double OFFSET = 8.25;
    private final int MINIMUM_MOVEMENT = 10;

    private DcMotor lift;      //motor used to control the lift
    private Servo pitchServo;
    private Servo yawServo;
    private Servo holdServo;
    private LinearOpMode myOpMode;
    private boolean showTelemetry     = false;
    private double setpointInches = 0;
    private double currentPosition = 0;
    private int lastPosition = 0;
    private boolean goingHome = false;
    private LiftStates currentState = HOME;
    private ElapsedTime stateTime = new ElapsedTime();
    private boolean sampleCollected = false;
    private ProportionalControl positionControl = new ProportionalControl(GAIN, ACCEL_LIMIT, OUTPUT_LIMIT, TOLERANCE, DEADBAND, false);

    // Arm Constructor
    public LiftSubsystem(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry){
        lift = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Requires motor encoder cables to be hooked up.
        pitchServo = myOpMode.hardwareMap.get(Servo.class, "pitch");
        yawServo = myOpMode.hardwareMap.get(Servo.class, "yaw");
        holdServo = myOpMode.hardwareMap.get(Servo.class, "hold");

        setBucketPosition(BucketPositions.HOME);
        homeTheLift();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void readSensors(){
        int encoderValue = lift.getCurrentPosition();
        currentPosition = (SLOPE * encoderValue) + OFFSET;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Lift Position", "%.1f inches", currentPosition);
        }
    }

    /**
     * stop the arm from moving
     */
    public void stop(){
        lift.setPower(0);
    }

    public void hold(){
        lift.setPower(HOLD_POWER);
    }

    public void sampleInBucket(){
        sampleCollected = true;
    }

    /**
     * controlling the motor and causing the lift to move to the setpoint
     */
    public void runControl() {
        readSensors();
        double error = setpointInches - currentPosition;
        double power = 0;

        // Decides if the lift should be homing, or if it is going to the correct position
        if(goingHome){

            // Decides if the arm is still in the homing motion or if it has stopped and is homed
            int position = lift.getCurrentPosition();
            if(Math.abs(position-lastPosition) < MINIMUM_MOVEMENT){
                power = 0;
                resetEncoders();
                goingHome = false;
            } else {
                power = HOME_POWER;
            }
            lastPosition = position;
            myOpMode.sleep(100);
            setSetpointInches(currentPosition);

        } else {

            // Controls the power when moving to the set point
            power = positionControl.getOutput(currentPosition);

            if(power == 0){
                hold();
            }
        }


        lift.setPower(power);
        myOpMode.telemetry.addData("lift error",error);
        myOpMode.telemetry.addData("lift power", power);
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getSetpointInches() {
        return setpointInches;
    }

    public void setSetpointInches(double setpointInches) {
        MathUtils.clamp(setpointInches, MIN_HEIGHT, MAX_HEIGHT);
        this.setpointInches = setpointInches;
        positionControl.reset(setpointInches);
    }

    public void resetEncoders(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(10);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void homeTheLift(){
        goingHome = true;
        myOpMode.telemetry.addLine("homing the lift");
        myOpMode.telemetry.update();
        lift.setPower(HOME_POWER);
        myOpMode.sleep(250);

    }

    public void setBucketPosition(BucketPositions position){
        switch (position){

            case HOME:{
                pitchServo.setPosition(0.5);
                yawServo.setPosition(0.5);
                holdServo.setPosition(1);
                break;
            }

            case HOME_READY:{
                pitchServo.setPosition(0.5);
                yawServo.setPosition(0.5);
                holdServo.setPosition(0.5);
                break;
            }

            case SIDE_DUMP_READY:{
                pitchServo.setPosition(0.4);
                yawServo.setPosition(0.7);
                holdServo.setPosition(0.5);
                break;
            }

            case SIDE_DUMP_RELEASE:{
                pitchServo.setPosition(0.4);
                yawServo.setPosition(0.7);
                holdServo.setPosition(0);
                break;
            }

            case BACK_DUMP_READY:{
                pitchServo.setPosition(0.7);
                yawServo.setPosition(0.5);
                holdServo.setPosition(0.5);
                break;
            }

            case BACK_DUMP_RELEASE:{
                pitchServo.setPosition(0.7);
                yawServo.setPosition(0.5);
                holdServo.setPosition(1);
                break;
            }
        }

    }

    public void runStateMachine () {

        if (showTelemetry) {
            myOpMode.telemetry.addData("Lift State", "%S", currentState);
        }

        switch (currentState) {

            case HOME:{
                if (sampleCollected){
                    setBucketPosition(BucketPositions.HOME_READY);

                    setState(SAMPLE_HELD);
                }
                break;
            }

            case SAMPLE_HELD:{
                if(myOpMode.gamepad2.triangle){
                    setSetpointInches(HIGH_BASKET);
                    setState(LIFTING);
                }
                break;
            }

            case LIFTING:{
                if(positionControl.inPosition()){
                    setBucketPosition(BucketPositions.BACK_DUMP_READY);
                    setState(READY_TO_SCORE);
                }
                break;
            }

            case READY_TO_SCORE:{
                if(myOpMode.gamepad2.cross){
                    setBucketPosition(BucketPositions.BACK_DUMP_RELEASE);
                    setState(DUMPED);
                }
                break;
            }

            case DUMPED:{
                if(stateTime.time() > 1){
                    setSetpointInches(MIN_HEIGHT);
                    setState(LOWERING);
                }
                break;
            }

            case LOWERING:{
                if(positionControl.inPosition()){
                    setBucketPosition(BucketPositions.HOME);
                    sampleCollected = false;
                    setState(HOME);
                }
                break;
            }

        }
    }

    public void setState (LiftStates newState){
        currentState = newState;
        stateTime.reset();

    }

}
