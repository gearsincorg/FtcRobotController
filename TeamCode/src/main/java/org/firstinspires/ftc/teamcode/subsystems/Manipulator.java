package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Manipulator {
    private static final double LIFT_GAIN       = 0.06;    // Strength of lift position control
    private static final double LIFT_TOLERANCE  = 1.0;      // Controller is "inPosition" if position error is < +/- this amount
    public static final double LIFT_HOME_ANGLE = 0.0;
    public static final double LIFT_HOVER_ANGLE = 10.0;
    public static final double LIFT_AUTO_ANGLE = 15.0;
    public static final double LIFT_FRONT_ANGLE = 40.0;
    public static final double LIFT_HANG_ANGLE  = 90.0;
    public static final double LIFT_BACK_ANGLE = 115.0;

    public static final double LIFT_MIN_ANGLE = 0.0;
    public static final double LIFT_MAX_ANGLE = 120.0;

    private static final double EXTEND_GAIN     = 0.3;    // Strength of extend position control
    private static final double EXTEND_TOLERANCE = 0.25;     // Controller is is "inPosition" if position error is < +/- this amount
    public static final double EXTEND_HOME_DISTANCE = 0.0;
    public static final double EXTEND_FRONT_DISTANCE = 10.0;
    public static final double EXTEND_BACK_DISTANCE = 6.0;
    public static final double EXTEND_MIN_LENGTH = 0.0;
    public static final double EXTEND_MAX_LENGTH = 19.5;

    private static final double LIFT_COUNTS_PER_DEGREE = 11.05556 ; // 995 counts for 90 Deg
    private static final double EXTEND_COUNTS_PER_INCH = 158.944 ;  // 2861 counts for 18"

    private static final double SHORT_HOLD_POWER = 0.21  ;
    private static final double LONG_HOLD_POWER  = 0.10  ;

    private static final double WRIST_HOME = 0.0;
    private static final double WRIST_SCORE_FRONT = 0.1;
    private static final double WRIST_SCORE_BACK = 0.675;

    private static final double GRAB_LEFT_AUTO   = 0.55;
    private static final double GRAB_LEFT_OPEN   = 0.50;
    private  static final double GRAB_LEFT_CLOSE = 0.25;

    private static final double GRAB_RIGHT_AUTO  = 0.45;
    private static final double GRAB_RIGHT_OPEN  = 0.50;
    private static final double GRAB_RIGHT_CLOSE = 0.73;

    private static final int MIN_PIXLE_HITS      = 5;

    public  double liftAngle      = 0;   // Arm angle in degrees.  Horizontal = 0 degrees.  Increases to approximately 120 degrees.
    public  double extendLength   = 0;

    public boolean pixelLeftInRange = false;
    public boolean pixelRightInRange = false;
    public boolean powerLiftingIsActive = false;

    // Hardware interface Objects
    private DcMotor lift;       //  control the arm Lift Motor
    private DcMotor extend;     //  control the Linear Slide Extension Motor
    private Servo wrist;        //  control the claw rotation wrist
    private Servo clawL;        //  control the left claw open/close
    private Servo clawR;        //  control the right claw open/close
    private DistanceSensor pixelL;
    private DistanceSensor pixelR;
//  private DistanceSensor frontRange;

    private int liftEncoder    = 0;
    private int extendEncoder  = 0;

    private boolean rangeEnabled = false;
    private double pixelLeftRange = 0;
    private int leftPixelCounter = 0;
    private double pixelRightRange = 0;
    private int rightPixelCounter = 0;
//    private double frontDistance = 0;

    private double  liftSetpoint  = 0;
    private boolean liftInPosition =false;
    private double  extendSetpoint   = 0;
    private boolean extendInPosition = false;
    private double  wristAngle    = 0;


    private ManipulatorState currentState = ManipulatorState.UNKNOWN;
    private ManipulatorState nextState    = ManipulatorState.UNKNOWN;
    private double stateDelay = 0.0;

    private LinearOpMode myOpMode;
    private boolean showTelemetry = false;
    private ElapsedTime armTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.
    private ElapsedTime stateTimer = new ElapsedTime();
    private double elapsedTime = 0;

    // Manipulator Constructor
    public Manipulator(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Manipulator Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry)
    {
        lift = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extend = myOpMode.hardwareMap.get(DcMotor.class, "extend");
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        clawL = myOpMode.hardwareMap.get(Servo.class, "left_claw");
        clawR = myOpMode.hardwareMap.get(Servo.class, "right_claw");
        pixelL = myOpMode.hardwareMap.get(DistanceSensor.class, "left_pixel");
        pixelR = myOpMode.hardwareMap.get(DistanceSensor.class, "right_pixel");
//        frontRange = myOpMode.hardwareMap.get(DistanceSensor.class, "front_range");

        // Do any cleanup in teleop
        if (!Globals.IS_AUTO) {

            // if we have not explicitly closed a grabber, make it safe by opening it (could be in auto state)
            if (!Globals.LEFT_GRABBER_CLOSED) {
                openLeftGrabber();
            }
            if (!Globals.RIGHT_GRABBER_CLOSED) {
                openRightGrabber();
            }
            currentState = Globals.ARM_STATE;
        }

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    /**
     *  Read all input devices to determine the robot's motion
     *  always return true so this can be used in while loop conditions
     */
    public boolean readSensors() {
        liftEncoder    = lift.getCurrentPosition() ;
        extendEncoder  = extend.getCurrentPosition() ;

        liftAngle = liftEncoder / LIFT_COUNTS_PER_DEGREE;
        extendLength = extendEncoder / EXTEND_COUNTS_PER_INCH;

        if (rangeEnabled) {
            pixelLeftRange = pixelL.getDistance(DistanceUnit.MM);
            pixelRightRange = pixelR.getDistance(DistanceUnit.MM);
        }

        if ((pixelLeftRange >= 20) && (pixelLeftRange <= 90)){
            if (leftPixelCounter++ > MIN_PIXLE_HITS) {
                pixelLeftInRange = true;
            }
        } else {
            pixelLeftInRange = false;
            leftPixelCounter = 0;
        }

        if ((pixelRightRange >= 20) && (pixelRightRange <= 90)){
            if (rightPixelCounter++ > MIN_PIXLE_HITS) {
                pixelRightInRange = true;
            }
        } else {
            pixelRightInRange = false;
            rightPixelCounter = 0;
        }

        //            frontDistance = frontRange.getDistance(DistanceUnit.MM);

        if (showTelemetry) {
            myOpMode.telemetry.addData("Arm Encoders L:X", "%6d %6d", liftEncoder, extendEncoder);
            myOpMode.telemetry.addData("Arm Pos L:X", "%5.1f %5.1f", liftAngle, extendLength);
            if (rangeEnabled) {
                myOpMode.telemetry.addData("Pixel L R:T", "%4.0f %s", pixelLeftRange, pixelLeftInRange ? "YES" : "No");
                myOpMode.telemetry.addData("Pixel R R:T", "%4.0f %s", pixelRightRange, pixelRightInRange ? "YES" : "No");
                // myOpMode.telemetry.addData("Front Range", "%4.0f ", frontDistance);
            }
        }

        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    /**
     * run one cycle all automated arm features
     */
    public void runArmControl(){
        readSensors();
        runLiftControl();
        runExtendControl();
        runStateMachine();
    }

    public void runArmControl(double holdTime){
        armTimer.reset();
        while(myOpMode.opModeIsActive() && (armTimer.time() < holdTime)){
            runArmControl();
            myOpMode.sleep(5);
        }
    }

    /**
     * Allow the copilot to adjust the arm lift and extend positions with the joystick
     */
    public void manualArmControl() {
        if (elapsedTime > 0) {
            double cycleTime = (armTimer.time() - elapsedTime);

            // Check for manual adjustments to arm positions.
            if (Math.abs(myOpMode.gamepad2.left_stick_y) > 0.25) {
                liftSetpoint += (-myOpMode.gamepad2.left_stick_y * cycleTime * 20);   //  max 20 degrees per second
                liftSetpoint = Range.clip(liftSetpoint, LIFT_MIN_ANGLE, LIFT_MAX_ANGLE);
            }

            if (Math.abs(myOpMode.gamepad2.right_stick_x) > 0.25) {
                extendSetpoint += (myOpMode.gamepad2.right_stick_x * cycleTime * 10.0) ;  // max 10 inches per second
                extendSetpoint = Range.clip(extendSetpoint, EXTEND_MIN_LENGTH, EXTEND_MAX_LENGTH);
            }
        }
        elapsedTime = armTimer.time();
    }

    public void enablePowerLifting() {
        lift.setPower(0);
        gotoHang();
    }

    public void disablePowerLifting() {
        powerLiftingIsActive = false;
    }

    public void powerLift(){
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setLiftPower(-0.25);
    }

    public void powerHold(){
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (liftAngle < 20) {   // holding vertical
            setLiftPower(-0.20);
        } else if (liftAngle < 87) {
            setLiftPower(0.10);
        } else if (liftAngle > 93) {
            setLiftPower(-0.10);
        } else {
            setLiftPower(0);
        }
    }

    public boolean weArePowerLifting() {
        return powerLiftingIsActive;
    }

    public void setRangeEnable(boolean enableRange) {
        rangeEnabled = enableRange;
    }

    /**
     * controlling lift power to get to the lift set point
     * @return true if arm is in position
     */
    public boolean runLiftControl() {

        // abort any action if wer are in final lifting.
        if (powerLiftingIsActive) {
            if (showTelemetry) {
                myOpMode.telemetry.addData("Power Lifting", "Is Enabled");
            }
            return true;
        }

        double error = liftSetpoint - liftAngle;
        double errorPower = error * LIFT_GAIN;
        double weightPower = SHORT_HOLD_POWER  + (LONG_HOLD_POWER * extendLength / EXTEND_MAX_LENGTH);
        double power = errorPower + (weightPower * Math.cos(Math.toRadians(liftAngle)));

        power = Range.clip(power, -0.4, 0.5);

        //causes the lift power to be zero if the arms angle is past a certain point
        if (((power < 0) && (liftAngle < 25)) ||
            ((power > 0) && (liftAngle > 110))){
            power = 0;
        }

        lift.setPower(power);
        if (showTelemetry) {
            myOpMode.telemetry.addData("Lift E:P", "%6.1f %6.2f", error, power);
        }

        liftInPosition = (Math.abs(error) < LIFT_TOLERANCE);
        return liftInPosition;
    }

    public boolean runExtendControl() {
        double error = extendSetpoint - extendLength;
        double power = error * EXTEND_GAIN;


        power = Range.clip(power, -1.0, 1.0);

        // causes the lift power to be zero if the extend is fully retracted
        if (power < 0){
            if (extendLength < 0.2) {
                power = -0.00;
            }
        }

        extend.setPower(power);
        if (showTelemetry) {
            myOpMode.telemetry.addData("Extend E:P", "%6.1f %6.2f", error, power);
        }

        extendInPosition =  (Math.abs(error) < EXTEND_TOLERANCE);
        return extendInPosition;
    }

    /**
     *
     * @param setpoint the target angle for the lift
     */
    public void setLiftSetpoint(double setpoint) {
        liftSetpoint = setpoint;
        liftInPosition = false;
    }

    public void setExtendSetpoint(double setpoint) {
        extendSetpoint = setpoint;
        extendInPosition = false;
    }

    public void setLiftPower(double power) {
        lift.setPower(power);
    }

    public void homeArm() {
        readSensors();
        int lastLiftPos   = liftEncoder;
        int lastExtendPos = extendEncoder;
        boolean liftIsHome = false;
        boolean extendIsHome = false;

        // Start retracting arm
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(-0.2);
        extend.setPower(-0.3);
        myOpMode.sleep(200);

        while((myOpMode.opModeInInit() || myOpMode.opModeIsActive()) && readSensors() && (!liftIsHome || !extendIsHome)) {
            if (Math.abs(lastLiftPos - liftEncoder) < 5) {
                lift.setPower(0);
                liftIsHome = true;
            }

            if (Math.abs(lastExtendPos - extendEncoder) < 5) {
                extend.setPower(0);
                extendIsHome = true;
            }

            lastLiftPos = liftEncoder;
            lastExtendPos = extendEncoder;

            myOpMode.sleep(100);
            myOpMode.telemetry.addData("Arm", "Homing");
            myOpMode.telemetry.update();
        }

        lift.setPower(0);
        extend.setPower(0);
        myOpMode.telemetry.addData("Arm", "Is Home");
        myOpMode.telemetry.update();

        // Reset encoders and determine lift and extend home positions.
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(100);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Globals.ARM_HAS_HOMED = true;
        setState(ManipulatorState.HOME);
    }

    public void update(){
        if (showTelemetry) {
        }
    }

    //------------ Servo Functions ------
    public void wristToPickupPosition() {
        wrist.setPosition(WRIST_HOME);
        clawL.setPosition(GRAB_LEFT_OPEN);
        clawR.setPosition(GRAB_RIGHT_OPEN);
        Globals.LEFT_GRABBER_CLOSED = false;
        Globals.RIGHT_GRABBER_CLOSED = false;
        Globals.WRIST_STATE = WristState.HOME;
    }

    public void closeLeftGrabber (){
        clawL.setPosition(GRAB_LEFT_CLOSE);
        Globals.LEFT_GRABBER_CLOSED = true;
    }
    public void openLeftGrabber (){
        clawL.setPosition(GRAB_LEFT_OPEN);
        Globals.LEFT_GRABBER_CLOSED = false;
    }
    public void closeRightGrabber (){
        clawR.setPosition(GRAB_RIGHT_CLOSE);
        Globals.RIGHT_GRABBER_CLOSED = true;
    }
    public void openRightGrabber (){
        clawR.setPosition(GRAB_RIGHT_OPEN);
        Globals.RIGHT_GRABBER_CLOSED = false;
    }
    public void openGrabbers (){
        clawL.setPosition(GRAB_LEFT_OPEN);
        clawR.setPosition(GRAB_RIGHT_OPEN);
        Globals.LEFT_GRABBER_CLOSED = false;
        Globals.RIGHT_GRABBER_CLOSED = false;
    }
    public void autoOpenGrabbers (){
        clawL.setPosition(GRAB_LEFT_AUTO);
        clawR.setPosition(GRAB_RIGHT_AUTO);
        Globals.LEFT_GRABBER_CLOSED = false;
        Globals.RIGHT_GRABBER_CLOSED = false;
    }

    public void wristToHome(){
        wrist.setPosition(WRIST_HOME);
        Globals.WRIST_STATE = WristState.HOME;
    }

    public void wristToFrontScore(){
        wrist.setPosition(WRIST_SCORE_FRONT);
        Globals.WRIST_STATE = WristState.FRONT_SCORE;
    }
    public void wristToBackScore(){
        wrist.setPosition(WRIST_SCORE_BACK);
        Globals.WRIST_STATE = WristState.BACK_SCORE;
    }

    //------------- state machine functions -------------
    private boolean smGotoHome       = false;
    private boolean smGotoSafeDriving = false;
    private boolean smGotoFrontScore  = false;
    private boolean smGotoBackScore  = false;

    public void gotoHome() {
        smGotoHome = true;
    }

    public void gotoSafeDriving() {
        smGotoSafeDriving = true;
    }

    public void gotoFrontScore() {
        smGotoFrontScore = true;
    }

    public void gotoBackScore() {
        smGotoBackScore = true;
    }

    public void gotoHang() {
        closeLeftGrabber();
        closeRightGrabber();
        setExtendSetpoint(EXTEND_HOME_DISTANCE);
        setState(ManipulatorState.TH_RETRACTING);
    }

    // State machine.
    public void runStateMachine(){

        if (showTelemetry) {
            myOpMode.telemetry.addData("State Machine", "%S", currentState);
        }

        switch (currentState){

            case UNKNOWN:
                break;

            // -- Home  ----------
            case H_ROTATE:
                wristToHome();
                setState(ManipulatorState.H_RETRACTING);
                break;

            case H_LIFTING:
                if (liftInPosition) {
                    wristToHome();
                    setStateWithDelay(ManipulatorState.H_RETRACTING, 1.0);
                }
                break;

            case H_RETRACTING:
                if (extendInPosition) {
                    smGotoSafeDriving = false;
                    smGotoFrontScore  = false;
                    smGotoBackScore  = false;
                    setLiftSetpoint(LIFT_HOME_ANGLE);
                    openGrabbers();
                    setState(ManipulatorState.HOME);
                }
                break;

            case HOME:
                smGotoHome = false;
                rangeEnabled = true;
                if (smGotoSafeDriving) {
                    rangeEnabled = false;
                    setLiftSetpoint(LIFT_HOVER_ANGLE);
                    setState(ManipulatorState.SD_LIFTING);
                } else if (smGotoFrontScore) {
                    setLiftSetpoint(LIFT_FRONT_ANGLE);
                    setState(ManipulatorState.FS_LIFTING);
                } else if (smGotoBackScore) {
                    rangeEnabled = false;
                    setLiftSetpoint(LIFT_BACK_ANGLE);
                    setStateWithDelay(ManipulatorState.BS_LIFTING, 0.25);
                }
                break;

            // -- Safe Driving  ----------
            case SD_LIFTING:
                if (liftInPosition){
                    wristToBackScore();
                    setStateWithDelay(ManipulatorState.SD_LOWER, 0.75);
                }
                break;

            case SD_LOWER:
                setLiftSetpoint(LIFT_HOME_ANGLE);
                closeLeftGrabber();
                closeRightGrabber();
                setState(ManipulatorState.SAFE_DRIVING);
                break;

            case SAFE_DRIVING:
                smGotoSafeDriving = false;
                if (smGotoHome) {
                    setLiftSetpoint(LIFT_HOVER_ANGLE);
                    setState(ManipulatorState.H_LIFTING);
                } else if (smGotoFrontScore) {
                    setLiftSetpoint(LIFT_FRONT_ANGLE);
                    setState(ManipulatorState.FS_LIFTING);
                } else if (smGotoBackScore) {
                    setLiftSetpoint(LIFT_BACK_ANGLE);
                    setState(ManipulatorState.BS_LIFTING);
                }
                break;

            // -- Front Score  ----------
            case FS_LIFTING:
                if (liftInPosition){
                    wristToFrontScore();
                    setStateWithDelay(ManipulatorState.FS_EXTEND, 0.5);
                }
                break;

            case FS_EXTEND:
                setExtendSetpoint(EXTEND_FRONT_DISTANCE);
                setState(ManipulatorState.FRONT_SCORE);
                break;

            case FRONT_SCORE:
                smGotoFrontScore = false;
                if (smGotoHome) {
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setStateWithDelay(ManipulatorState.H_ROTATE, 0.5);
                } else if (smGotoSafeDriving) {
                    wristToBackScore();
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setStateWithDelay(ManipulatorState.SD_LOWER, 0.5);
                } else if (smGotoBackScore) {
                    setLiftSetpoint(LIFT_BACK_ANGLE);
                    setState(ManipulatorState.BS_LIFTING);
                }
                break;

            // -- Back Score  ----------
            case BS_LIFTING:
                wristToBackScore();
                if (liftInPosition){
                    setStateWithDelay(ManipulatorState.BS_EXTEND, 0.25);
                }
                break;

            case BS_EXTEND:
                setExtendSetpoint(EXTEND_BACK_DISTANCE);
                setState(ManipulatorState.BACK_SCORE);
                break;

            case BACK_SCORE:
                smGotoBackScore = false;
                if (smGotoHome) {
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setStateWithDelay(ManipulatorState.H_ROTATE, 0.5);
                } else if (smGotoSafeDriving) {
                    wristToBackScore();
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setStateWithDelay(ManipulatorState.SD_LOWER, 0.5);
                }  else if (smGotoFrontScore) {
                    wristToBackScore();
                    setExtendSetpoint(EXTEND_FRONT_DISTANCE);
                    setState(ManipulatorState.FS_LIFTING);
                }
                break;

            case TH_RETRACTING:
                if(extendInPosition) {
                    setLiftSetpoint(LIFT_HANG_ANGLE);
                    wristToBackScore();
                    setState(ManipulatorState.TH_LIFTING);
                }
                break;

            case TH_LIFTING:
                if(liftInPosition) {
                    setStateWithDelay(ManipulatorState.TRUSS_HANG, 0.5);
                }
                break;

            case TRUSS_HANG:
                powerLiftingIsActive = true;
                break;

            case WAITING:
                // execute a delay before progressing to next state.
                if (stateTimer.time() >= stateDelay) {
                    setState(nextState);
                }
                break;
        }
    }

    public void setStateWithDelay(ManipulatorState newState, double delaySec) {
        stateDelay = delaySec;
        nextState = newState;
        setState(ManipulatorState.WAITING);
    }

    public void setState(ManipulatorState newState){
        currentState = newState;
        Globals.ARM_STATE = newState;
        stateTimer.reset();
    }
}
