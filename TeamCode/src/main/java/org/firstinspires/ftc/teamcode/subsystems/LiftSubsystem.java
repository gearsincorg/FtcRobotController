package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.AUTO_WAIT;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.DUMPED;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.HOME;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.LIFTING;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.LOWERING;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.RDY_TO_DUMP;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.SAMPLE_HELD;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.SLOW_DUMP;
import static org.firstinspires.ftc.teamcode.subsystems.LiftStates.WAIT_BUCKET;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftSubsystem {

    // Standard SubSystem Members:
    private LinearOpMode myOpMode;
    private boolean     showTelemetry   = false;
    private LiftStates  currentState    = HOME;
    private double      outputPower     = 0;
    private ElapsedTime stateTime       = new ElapsedTime();

    // Constants
    public final double MAX_HEIGHT = 46;
    public final double MIN_HEIGHT = 8.5;
    public final double HIGH_BASKET = 46.0;
    public final double LOW_BASKET = 31 ;
    private final double HOLD_POWER = 0.2; // 0.075
    private final double HOME_POWER = -0.6;

    private final double TILT_BUCKET_READY = 0.525;
    private final double TILT_BACK = 0.48;
    private final double TILT_DUMP = 0.33;

    private final double GAIN = 0.8;
    private final double ACCEL_LIMIT = 6.0;
    private final double OUTPUT_LIMIT = 1;
    private final double TOLERANCE = 1.5;
    private final double DEADBAND = 0.25;

    private final double SLOPE = 0.0123;
    private final double OFFSET = 8.25;
    private final int MINIMUM_MOVEMENT = 10;

    private DcMotorEx lift;      //motor used to control the lift
    private Servo pitchServo;
    private Servo yawServo;
    private Servo holdServo;

    // Private Members
    private double  currentPosition = 0;
    private int     lastPosition = 0;
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
        lift = myOpMode.hardwareMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorEx.Direction.REVERSE);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset Encoders to zero
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);    // Still Requires motor encoder cables to be hooked up.
        pitchServo = myOpMode.hardwareMap.get(Servo.class, "pitch");
        yawServo = myOpMode.hardwareMap.get(Servo.class, "yaw");
        holdServo = myOpMode.hardwareMap.get(Servo.class, "hold");

        setBucketPosition(BucketPositions.HOME);

        if (!Globals.LIFT_HOMED) {
            homeTheLift();
        }

        readSensors();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void update() {
        readSensors();
        runControl();
        runStateMachine();

        if (showTelemetry) {
            myOpMode.telemetry.addData("LIFT Pos SP Pwr", "%s %.1f %.1f %.2f", currentState, currentPosition, positionControl.getSetPoint(), outputPower);
            myOpMode.telemetry.addData("Lift VEL", "%.0f", lift.getVelocity());
            //  myOpMode.telemetry.addData("Lift Pos", currentPosition);
            //  myOpMode.telemetry.addData("Lift Pwr", outputPower * 1000);
            //  myOpMode.telemetry.addData("Lift SP",  positionControl.getSetPoint());
        }
    }

    public void readSensors(){
        int encoderValue = lift.getCurrentPosition();
        currentPosition = (SLOPE * encoderValue) + OFFSET;

    }

    public void setSetpointInches(double setpointInches) {
        MathUtils.clamp(setpointInches, MIN_HEIGHT, MAX_HEIGHT);
        positionControl.reset(setpointInches);
    }

    public double getSetpointInches() {
        return positionControl.getSetPoint();
    }

    public void setBucketPosition(BucketPositions position){
        switch (position){

            default:
            case HOME:{
                pitchServo.setPosition(TILT_BUCKET_READY);
                break;
            }

            case BACK_DUMP_READY:{
                pitchServo.setPosition(TILT_BACK);
                break;
            }

            case BACK_DUMP_RELEASE:{
                pitchServo.setPosition(TILT_DUMP);
                break;
            }
        }
    }

    public void runStateMachine () {

        switch (currentState) {
            case HOME:{
                if((stateTime.time() > 0.5)) {
                    resetEncoders();
                    if (Globals.IS_AUTO) {
                        setState(AUTO_WAIT);
                    } else {
                        setState(SAMPLE_HELD);
                    }
                }
                break;
            }

            case AUTO_WAIT:{
                break;
            }

            case SAMPLE_HELD:{
                if (Globals.IS_AUTO && Globals.DID_NOT_SWEEP_SAMPLE) {
                    // bypass the scoring process as we don't have a sample
                    Globals.DID_NOT_SWEEP_SAMPLE = false;
                    setState(LOWERING);
                } else if(myOpMode.gamepad2.triangle || Globals.IS_AUTO){
                    setSetpointInches(HIGH_BASKET);
                    setBucketPosition(BucketPositions.BACK_DUMP_READY);
                    setState(LIFTING);
                } else if(myOpMode.gamepad2.square){
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
                    setState(RDY_TO_DUMP);
                }
                break;
            }

            case RDY_TO_DUMP:{
                if(myOpMode.gamepad2.cross || Globals.IS_AUTO){
                    setBucketPosition(BucketPositions.BACK_DUMP_RELEASE);
                    if (getSetpointInches() == MIN_HEIGHT){
                        setState(SLOW_DUMP);
                    } else {
                        setState(DUMPED);
                    }
                } else if(myOpMode.gamepad2.triangle){
                    setSetpointInches(HIGH_BASKET);
                    setBucketPosition(BucketPositions.BACK_DUMP_READY);
                    setState(LIFTING);
                } else if(myOpMode.gamepad2.square){
                    setSetpointInches(LOW_BASKET);
                    setBucketPosition(BucketPositions.BACK_DUMP_READY);
                    setState(LIFTING);
                }
                break;
            }

            case SLOW_DUMP:
                if(stateTime.time() > 0.2){
                    setState(DUMPED);
                }
                break;

            case DUMPED:{
                if(stateTime.time() > 0.5) {  // was .75
                    setBucketPosition(BucketPositions.HOME);
                    setState(WAIT_BUCKET);
                }
                break;
            }

            case WAIT_BUCKET:{
                // Wait till bucket returned OR In Auto, OR driver starts moving away
                if((stateTime.time() > 0.75) || Globals.IS_AUTO ||
                   ((Math.abs(Globals.DRIVE_AXIAL) + Math.abs(Globals.DRIVE_LATERAL)) > 0.25)){
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
        myOpMode.telemetry.addLine("Homing the lift");
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
        outputPower = 0;

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
                outputPower = 0;
                resetEncoders();
                goingHome = false;
                Globals.LIFT_HOMED = true;
                setBucketPosition(BucketPositions.HOME);
                setState(HOME);
            } else {
                outputPower = HOME_POWER;
            }
            lastPosition = position;
            myOpMode.sleep(50);
            setSetpointInches(currentPosition);

        } else {
            // Controls the outputPower when moving to the set point
            outputPower = positionControl.getOutput(currentPosition);

            if ((outputPower == 0) && (positionControl.getSetPoint() > MIN_HEIGHT)){
                outputPower = HOLD_POWER;
            }
        }

        lift.setPower(outputPower);
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

    public Action actionWaitForHomeOrState(LiftStates state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                return ((currentState != state) && (currentState != HOME));
            }
        };
    }
}

