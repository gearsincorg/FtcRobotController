package org.firstinspires.ftc.teamcode;

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
    public static final double LIFT_FRONT_ANGLE = 30.0;
    public static final double LIFT_BACK_ANGLE = 120.0;

    private static final double EXTEND_GAIN     = 0.2;    // Strength of extend position control
    private static final double EXTEND_TOLERANCE = 0.25;     // Controller is is "inPosition" if position error is < +/- this amount
    public static final double EXTEND_HOME_DISTANCE = 0.0;
    public static final double EXTEND_FRONT_DISTANCE = 6.0;
    public static final double EXTEND_BACK_DISTANCE = 6.0;
    public static final double MAX_EXTEND_LENGTH = 19.0;

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
    private static final double GRAB_RIGHT_CLOSE = 0.65;

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
    private DistanceSensor frontRange;

    private int liftEncoder    = 0;
    private int extendEncoder  = 0;

    private boolean rangeEnabled = false;
    private double pixelLeftRange = 0;
    private double pixelRightRange = 0;
    private double frontDistance = 0;

    private double  liftSetpoint  = 0;
    private boolean liftInPosition =false;
    private double  extendSetpoint   = 0;
    private boolean extendInPosition = false;
    private boolean clawLClosed   = false;
    private boolean clawRClosed   = false;
    private double  wristAngle    = 0;


    private ManipulatorState currentState = ManipulatorState.HOME;
    private ManipulatorState nextState    = ManipulatorState.HOME;
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
        frontRange = myOpMode.hardwareMap.get(DistanceSensor.class, "front_range");

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
            pixelLeftInRange = (pixelLeftRange > 20) && (pixelLeftRange < 65);
            pixelRightRange = pixelR.getDistance(DistanceUnit.MM);
            pixelRightInRange = (pixelRightRange > 20) && (pixelRightRange < 65);
        }

        if (showTelemetry) {
            myOpMode.telemetry.addData("Arm Encoders L:X", "%6d %6d", liftEncoder, extendEncoder);
            myOpMode.telemetry.addData("Arm Pos L:X", "%5.1f %5.1f", liftAngle, extendLength);
            if (rangeEnabled) {
                myOpMode.telemetry.addData("Pixel L R:T", "%4.0f %s", pixelLeftRange, pixelLeftInRange ? "YES" : "No");
                myOpMode.telemetry.addData("Pixel R R:T", "%4.0f %s", pixelRightRange, pixelRightInRange ? "YES" : "No");
            }
        }

        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    /**
     * Allow the copilot to adjust the arm lift and extend positions with the joystick
     */
    public void manualArmControl() {
        if (elapsedTime > 0) {
            double cycleTime = (armTimer.time() - elapsedTime);

            // Check for manual adjustments to arm positions.

            if (Math.abs(myOpMode.gamepad2.left_stick_y) > 0.25) {
                liftSetpoint += (-myOpMode.gamepad2.left_stick_y * cycleTime * 10);   //  max 10 degrees per second
            }

            if (Math.abs(myOpMode.gamepad2.right_stick_x) > 0.25) {
                extendSetpoint += (myOpMode.gamepad2.right_stick_y * cycleTime * 2.0) ;  // max 2 inches per second
            }
        }
        elapsedTime = armTimer.time();
    }

    public void enablePowerLifting() {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerLiftingIsActive = true;
    }

    public void disablePowerLifting() {
        powerLiftingIsActive = false;
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        double weightPower = SHORT_HOLD_POWER  + (LONG_HOLD_POWER * extendLength / MAX_EXTEND_LENGTH);
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
        int lastLiftPos   = liftEncoder;
        int lastExtendPos = extendEncoder;
        boolean liftIsHome = false;
        boolean extendIsHome = false;

        // Start retracting arm
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(-0.2);
        extend.setPower(-0.2);
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
        clawLClosed = false;
        clawRClosed = false;
    }
    public void closeLeftGrabber (){
        clawL.setPosition(GRAB_LEFT_CLOSE);
        clawLClosed = true;
    }
    public void openLeftGrabber (){
        clawL.setPosition(GRAB_LEFT_OPEN);
        clawLClosed = false;
    }
    public void closeRightGrabber (){
        clawR.setPosition(GRAB_RIGHT_CLOSE);
        clawRClosed = true;
    }
    public void openRightGrabber (){
        clawR.setPosition(GRAB_RIGHT_OPEN);
        clawRClosed = false;
    }
    public void openGrabbers (){
        clawL.setPosition(GRAB_LEFT_OPEN);
        clawR.setPosition(GRAB_RIGHT_OPEN);
        clawLClosed = false;
        clawRClosed = false;
    }
    public void autoOpenGrabbers (){
        clawL.setPosition(GRAB_LEFT_AUTO);
        clawR.setPosition(GRAB_RIGHT_AUTO);
        clawLClosed = false;
        clawRClosed = false;
    }

    public void wristToHome(){
        wrist.setPosition(WRIST_HOME);
    }
    public void wristToFrontScore(){
        wrist.setPosition(WRIST_SCORE_FRONT);
    }
    public void wristToBackScore(){
        wrist.setPosition(WRIST_SCORE_BACK);
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

    // State machine.
    public void runStateMachine(){

        if (showTelemetry) {
            myOpMode.telemetry.addData("State Machine", "%S", currentState);
        }

        switch (currentState){

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
                    rangeEnabled = false;
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
                }
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
        stateTimer.reset();
    }
}
