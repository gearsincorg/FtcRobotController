package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.AUTO_WAITING;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.DUMPED;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.HOME;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.LIFTING;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.LOWERING;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.READY_TO_SCORE;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.SAMPLE_HELD;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.WAITING_FOR_BUCKET;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftSubsystem {

    // Standard SubSystem Members:
    private LinearOpMode myOpMode;
    private boolean     showTelemetry   = false;
    private LiftStates  currentState    = HOME;
    private ElapsedTime stateTime       = new ElapsedTime();

    // Constants
    public final double MAX_HEIGHT = 47;
    public final double MIN_HEIGHT = 8.5;
    public final double HIGH_BASKET = 46.5;
    public final double LOW_BASKET = 25 ;
    private final double HOLD_POWER = 0.075;
    private final double HOME_POWER = -0.6;
    private final double TILT_SIDE = 0.4;
    private final double YAW_SIDE = 0.7;
    private final double TILT_BUCKET_READY = 0.535;
    private final double YAW_BUCKET_READY = 0.5;
    private final double TILT_BACK = 0.65;
    private final double YAW_BACK = 0.5;
    private final double HELD = 0.5;
    private final double OPENA = 0;
    private final double OPENB = 1;

    private final double GAIN = 0.8;
    private final double ACCEL_LIMIT = 6.0;
    private final double OUTPUT_LIMIT = 1;
    private final double TOLERANCE = 1.5;
    private final double DEADBAND = 0.25;

    private final double SLOPE = 0.0123;
    private final double OFFSET = 8.25;
    private final int MINIMUM_MOVEMENT = 10;

    private DcMotor lift;      //motor used to control the lift
    private Servo pitchServo;
    private Servo yawServo;
    private Servo holdServo;

    // Private Members
    private double setpointInches = 0;
    private double currentPosition = 0;
    private int lastPosition = 0;
    private boolean goingHome = false;
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
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset Encoders to zero
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // Still Requires motor encoder cables to be hooked up.
        pitchServo = myOpMode.hardwareMap.get(Servo.class, "pitch");
        yawServo = myOpMode.hardwareMap.get(Servo.class, "yaw");
        holdServo = myOpMode.hardwareMap.get(Servo.class, "hold");

        setBucketPosition(BucketPositions.HOME);
        homeTheLift();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void update() {
        readSensors();
        runControl();
        runStateMachine();
    }

    public void readSensors(){
        int encoderValue = lift.getCurrentPosition();
        currentPosition = (SLOPE * encoderValue) + OFFSET;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Lift Position", "%.1f inches", currentPosition);
            myOpMode.telemetry.addData("lift target  ", "%.1f inches", positionControl.getSetPoint());
        }
    }

    public void setSetpointInches(double setpointInches) {
        MathUtils.clamp(setpointInches, MIN_HEIGHT, MAX_HEIGHT);
        this.setpointInches = setpointInches;
        positionControl.reset(setpointInches);
    }

    public void setBucketPosition(BucketPositions position){
        switch (position){

            case HOME:{
                pitchServo.setPosition(TILT_BUCKET_READY);
                yawServo.setPosition(YAW_BUCKET_READY);
                holdServo.setPosition(HELD);
                break;
            }

            case HOME_READY:{
                pitchServo.setPosition(TILT_BUCKET_READY);
                yawServo.setPosition(YAW_BUCKET_READY);
                holdServo.setPosition(HELD);
                break;
            }

            case SIDE_DUMP_READY:{
                pitchServo.setPosition(TILT_SIDE);
                yawServo.setPosition(YAW_SIDE);
                holdServo.setPosition(HELD);
                break;
            }

            case SIDE_DUMP_RELEASE:{
                pitchServo.setPosition(TILT_SIDE);
                yawServo.setPosition(YAW_SIDE);
                holdServo.setPosition(OPENA);
                break;
            }

            case BACK_DUMP_READY:{
                pitchServo.setPosition(TILT_BACK);
                yawServo.setPosition(YAW_BACK);
                holdServo.setPosition(HELD);
                break;
            }

            case BACK_DUMP_RELEASE:{
                pitchServo.setPosition(TILT_BACK);
                yawServo.setPosition(YAW_BACK);
                holdServo.setPosition(OPENB);
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
                if((stateTime.time() > 0.5)) {
                    resetEncoders();
                    if (Globals.IS_AUTO) {
                        setState(AUTO_WAITING);
                    } else {
                        setState(SAMPLE_HELD);
                    }
                }
                break;
            }

            case AUTO_WAITING:{
                break;
            }

            case SAMPLE_HELD:{
                if(myOpMode.gamepad2.triangle || Globals.IS_AUTO){
                    setSetpointInches(HIGH_BASKET);
                    setBucketPosition(BucketPositions.BACK_DUMP_READY);
                    setState(LIFTING);
                } else if(myOpMode.gamepad2.circle){
                    setSetpointInches(LOW_BASKET);
                    setBucketPosition(BucketPositions.BACK_DUMP_READY);
                    setState(LIFTING);
                } else if(myOpMode.gamepad2.cross){
                    setBucketPosition(BucketPositions.BACK_DUMP_RELEASE);
                    setState(DUMPED);
                }
                break;
            }

            case LIFTING:{
                if(positionControl.inPosition()){
                    setState(READY_TO_SCORE);
                }
                break;
            }

            case READY_TO_SCORE:{
                if(myOpMode.gamepad2.cross || Globals.IS_AUTO){
                    setBucketPosition(BucketPositions.BACK_DUMP_RELEASE);
                    setState(DUMPED);
                }
                break;
            }

            case DUMPED:{
                if(stateTime.time() > 0.75){
                    setBucketPosition(BucketPositions.HOME);
                    setState(WAITING_FOR_BUCKET);
                }
                break;
            }

            case WAITING_FOR_BUCKET:{
                if(stateTime.time() > 0.75){
                    setSetpointInches(MIN_HEIGHT);
                    setState(LOWERING);
                }
               break;
            }

            case LOWERING:{
                if(positionControl.inPosition()){
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

    public void homeTheLift(){
        goingHome = true;
        myOpMode.telemetry.addLine("homing the lift");
        myOpMode.telemetry.update();
        lift.setPower(HOME_POWER);
        myOpMode.sleep(100);
    }

    public void resetEncoders(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(10);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * controlling the motor and causing the lift to move to the setpoint
     */
    public void runControl() {
        double power = 0;

        // do sanity check on setpoint
        if (positionControl.getSetPoint() < MIN_HEIGHT){
            setSetpointInches(MIN_HEIGHT);
        } else if (positionControl.getSetPoint() > HIGH_BASKET){
            setSetpointInches(HIGH_BASKET);
        }

        // Decides if the lift should be homing, or if it is going to the correct position
        if(goingHome){

            // Decides if the lift is still in the homing motion or if it has stopped and is homed
            int position = lift.getCurrentPosition();
            if(Math.abs(position-lastPosition) < MINIMUM_MOVEMENT){
                power = 0;
                resetEncoders();
                goingHome = false;
                setBucketPosition(BucketPositions.HOME_READY);
                setState(HOME);
            } else {
                power = HOME_POWER;
            }
            lastPosition = position;
            myOpMode.sleep(50);
            setSetpointInches(currentPosition);

        } else {
            // Controls the power when moving to the set point
            power = positionControl.getOutput(currentPosition);

            if (power == 0){
                power = HOLD_POWER;
            }
        }

        lift.setPower(power);
        if (showTelemetry) {
            myOpMode.telemetry.addData("lift power", power);
        }
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

    public Action actionSetState(LiftStates state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                setState(state);
                return false;
            }
        };
    }

    public Action actionWaitForState(LiftStates state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                return currentState != state;
            }
        };
    }
}

