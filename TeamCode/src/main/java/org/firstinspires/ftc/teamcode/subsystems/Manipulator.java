package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ManipulatorState.BACK_SCORE;
import static org.firstinspires.ftc.teamcode.subsystems.ManipulatorState.FRONT_SCORE;
import static org.firstinspires.ftc.teamcode.subsystems.ManipulatorState.H_RETRACTING;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Manipulator {
    private static final double LIFT_GAIN           =  0.07;    // Strength of lift position control  was 0.06
    private static final double LIFT_TOLERANCE      =  1.75;    // Controller is "inPosition" if position error is < +/- this amount
    public static final double LIFT_HOME_ANGLE      =  0.0;
    public static final double LIFT_STACK_PICK_2_3  =  4.0;
    public static final double LIFT_STACK_PICK_4_5  =  8.0;
    public static final double LIFT_STACK_LEVEL5    =  9.0;
    public static final double LIFT_LOW_AUTO_ANGLE  = 19.0;
    public static final double LIFT_HIGH_AUTO_ANGLE = 23.0;
    public static final double LIFT_FRONT_ANGLE     = 27.0;
    public static final double LIFT_HANG_ANGLE      = 82.0;
    public static final double LIFT_BACK_ANGLE      =114.0;
    public static final double SAFE_LIFT_ANGLE = 12.0;

    public static final double LIFT_MIN_ANGLE       = 0.0;
    public static final double LIFT_MAX_ANGLE       = 120.0;

    public static final double LIFT_MIN_POWER       = -0.6;
    public static final double LIFT_MAX_POWER       =  0.6 ;

    private static final double EXTEND_GAIN         =  0.3;    // Strength of extend position control
    private static final double EXTEND_TOLERANCE    =  0.50;   // Controller is is "inPosition" if position error is < +/- this amount
    public static final double EXTEND_HOME_DISTANCE =  0.0;
    public static final double EXTEND_LOW_AUTO_DISTANCE  = 5.5;
    public static final double EXTEND_HIGH_AUTO_DISTANCE = 7.5;
    public static final double EXTEND_FRONT_DISTANCE = 7.0;
    public static final double EXTEND_BACK_DISTANCE =  0.0;
    public static final double EXTEND_LIFT_LENGTH   = 19.2;
    public static final double EXTEND_MIN_LENGTH    =  0.0;
    public static final double EXTEND_MAX_LENGTH    = 19.5;
    public static final double SAFE_EXTEND_DISTANCE =  5.0;

    private static final double LIFT_COUNTS_PER_DEGREE = 11.05556 ; // 995 counts for 90 Deg
    private static final double EXTEND_COUNTS_PER_INCH = 158.944 ;  // 2861 counts for 18"

    private static final double MIN_PIXEL_RANGE_DETECT = 18 ;
    private static final double MAX_PIXEL_RANGE_DETECT = 85 ;

    private static final double SHORT_HOLD_POWER = 0.25  ;
    private static final double LONG_HOLD_POWER  = 0.15  ;

    private static final double WRIST_POS_HOME       = 0.08 ;  // was 0.00
    private static final double WRIST_POS_SCORE_BACK = 0.75;  // was 0.66
    private static final double WRIST_DEGREE_SCALE   = (WRIST_POS_SCORE_BACK - WRIST_POS_HOME) / 180;

    private static final double WRIST_HOME_ABS        =   0;
    private static final double WRIST_SCORE_FRONT_REL =  65;
    private static final double WRIST_HANG_ABS        = 140;
    private static final double WRIST_SCORE_BACK_ABS  = 180;

    private static final double GRAB_LEFT_AUTO   = 0.62;  // was .60
    private static final double GRAB_LEFT_OPEN   = 0.50;
    private  static final double GRAB_LEFT_CLOSE = 0.25;

    private static final double GRAB_RIGHT_AUTO  = 0.38;  // was .40
    private static final double GRAB_RIGHT_OPEN  = 0.50;
    private static final double GRAB_RIGHT_CLOSE = 0.73;

    private static final double STACK_WACKER_DOWN = 0.0;
    private static final double STACK_WACKER_UP   = 0.75;

    private static final int MIN_PIXEL_HITS = 2;

    public  double liftAngle      = 0;   // Arm angle in degrees.  Horizontal = 0 degrees.  Increases to approximately 120 degrees.
    public  double extendLength   = 0;

    public boolean pixelLeftInRange = false;
    public boolean pixelRightInRange = false;
    public double  wristOffset = 0;

    // members to track last arm positions in front and back
    public double  lastBackExtend;
    public double  lastFrontLift;
    public double  lastFrontExtend;

    // Hardware interface Objects
    private DcMotor lift;       //  control the arm Lift Motor
    private DcMotor extend;     //  control the Linear Slide Extension Motor
    private Servo wrist;        //  control the claw rotation wrist
    private Servo clawL;        //  control the left claw open/close
    private Servo clawR;        //  control the right claw open/close
    private Servo stackWacker;  //  control the rear StackWacker
    private DistanceSensor pixelL;
    private DistanceSensor pixelR;

    private int liftEncoder    = 0;
    private int extendEncoder  = 0;

    private boolean rangeEnabled = false;
    private double pixelLeftRange = 0;
    private int leftPixelCounter = 0;
    private double pixelRightRange = 0;
    private int rightPixelCounter = 0;

    private double  liftSetpoint  = 0;
    private boolean liftInPosition =false;
    private double  extendSetpoint   = 0;
    private boolean extendInPosition = false;

    private ManipulatorState currentState = ManipulatorState.UNKNOWN;
    private ManipulatorState nextState    = ManipulatorState.UNKNOWN;
    private double stateDelay = 0.0;

    private final LinearOpMode myOpMode;
    private final ElapsedTime armTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.
    private final ElapsedTime stateTimer = new ElapsedTime();
    private boolean showTelemetry = false;
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
        stackWacker = myOpMode.hardwareMap.get(Servo.class, "whacker");
        pixelL = myOpMode.hardwareMap.get(DistanceSensor.class, "left_pixel");
        pixelR = myOpMode.hardwareMap.get(DistanceSensor.class, "right_pixel");

        lastBackExtend  = EXTEND_BACK_DISTANCE;
        lastFrontLift   = LIFT_FRONT_ANGLE;
        lastFrontExtend = EXTEND_FRONT_DISTANCE;

        // Do any cleanup in teleop
        if (!Globals.IS_AUTO) {

            // if we have not explicitly closed a grabber, make it safe by opening it (could be in auto state)
            if (!Globals.HOLDING_LEFT_PIXEL) {
                openLeftGrabber();
            }
            if (!Globals.HOLDING_RIGHT_PIXEL) {
                openRightGrabber();
            }
            currentState = Globals.ARM_STATE;
        }

        stackWackerUp();

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

            if ((pixelLeftRange >= MIN_PIXEL_RANGE_DETECT) && (pixelLeftRange <= MAX_PIXEL_RANGE_DETECT)){
                if (leftPixelCounter++ > MIN_PIXEL_HITS) {
                    pixelLeftInRange = true;
                }
            } else {
                pixelLeftInRange = false;
                leftPixelCounter = 0;
            }

            if ((pixelRightRange >= MIN_PIXEL_RANGE_DETECT) && (pixelRightRange <= MAX_PIXEL_RANGE_DETECT)){
                if (rightPixelCounter++ > MIN_PIXEL_HITS) {
                    pixelRightInRange = true;
                }
            } else {
                pixelRightInRange = false;
                rightPixelCounter = 0;
            }
        } else {
            pixelLeftInRange = false;
            pixelRightInRange = false;
            leftPixelCounter = 0;
            rightPixelCounter = 0;
        }

        if (showTelemetry) {
            // myOpMode.telemetry.addData("Arm ENC L:X", "%5d %5d", liftEncoder, extendEncoder);
            myOpMode.telemetry.addData("Arm Pos L:X", "%5.1f %5.1f", liftAngle, extendLength);
            if (rangeEnabled) {
                myOpMode.telemetry.addData("Pixel L R:T", "%4.0f %s", pixelLeftRange, pixelLeftInRange ? "YES" : "No");
                myOpMode.telemetry.addData("Pixel R R:T", "%4.0f %s", pixelRightRange, pixelRightInRange ? "YES" : "No");
                // myOpMode.telemetry.addData("Front Range", "%4.0f ", frontDistance);
            }
        }

        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    public void setRangeEnable(boolean enableRange) {
        rangeEnabled = enableRange;
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

    public void waitTillArmInPosition() {
        while (myOpMode.opModeIsActive() && !(liftInPosition && extendInPosition)) {
            runArmControl();
            myOpMode.sleep(1);
        }
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

            // Check for manual adjustments to lift angle.
            if (Math.abs(myOpMode.gamepad2.left_stick_y) > 0.25) {
                liftSetpoint += (-myOpMode.gamepad2.left_stick_y * cycleTime * 20);   //  max 20 degrees per second
                liftSetpoint = Range.clip(liftSetpoint, LIFT_MIN_ANGLE, LIFT_MAX_ANGLE);

                // should we save this for next Front
                if (currentState == FRONT_SCORE) {
                    lastFrontLift = liftSetpoint;
                }
            }

            // Check for manual adjustments to extend distance.
            if (Math.abs(myOpMode.gamepad2.right_stick_y) > 0.25) {
                extendSetpoint += (-myOpMode.gamepad2.right_stick_y * cycleTime * 15.0) ;  // max 10 inches per second
                extendSetpoint = Range.clip(extendSetpoint, EXTEND_MIN_LENGTH, EXTEND_MAX_LENGTH);

                // should we save this for next Front/Back Score
                if (currentState == BACK_SCORE) {
                    lastBackExtend = extendSetpoint;
                } else if (currentState == FRONT_SCORE) {
                    lastFrontExtend = extendSetpoint;
                }
            }
        }
        elapsedTime = armTimer.time();
    }

    public void powerLift(){
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setLiftPower(-0.4);
    }

    public void powerHold(){
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (liftAngle < 20) {   // holding robot tilted vertical
            setLiftPower(-0.20);
        } else {
            setLiftPower((liftSetpoint - liftAngle) * LIFT_GAIN);
        }
    }

    /**
     * controlling lift power to get to the lift set point
     */
    public void runLiftControl() {

        // abort any action if w er are in final lifting.
        if (readyToHang()) {
            if (showTelemetry) {
                myOpMode.telemetry.addData("Power Lifting", "Is Active");
            }
            return;
        }

        double error = liftSetpoint - liftAngle;
        double errorPower = error * LIFT_GAIN;
        double weightPower = SHORT_HOLD_POWER  + (LONG_HOLD_POWER * extendLength / EXTEND_MAX_LENGTH);
        double power = errorPower + (weightPower * Math.cos(Math.toRadians(liftAngle)));

        power = Range.clip(power, LIFT_MIN_POWER, LIFT_MAX_POWER);  /// was -0.4 & 0.5

        //causes the lift power to be zero (break) if the arms angle is past a certain point, or sitting at home
        if (((liftSetpoint < 1) && (liftAngle < 5)) ||
            ((power < 0) && (liftAngle < 25)) ||
            ((power > 0) && (liftAngle > 100))
            ){
            power = 0;
        } else if ((liftSetpoint < 12) && (error > -3.0) && (error < -0.2)){
            power = power / 2;
        }

        lift.setPower(power);
        if (showTelemetry) {
            myOpMode.telemetry.addData("Lift E:P", "%6.1f %6.2f", error, power);
        }

        liftInPosition = (Math.abs(error) < LIFT_TOLERANCE);
    }

    public void runExtendControl() {
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
        int lastLiftEnc   = liftEncoder;
        int lastExtendEnc = extendEncoder;
        boolean liftIsHome = false;
        boolean extendIsHome = false;

        // Start retracting arm
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(-0.2);
        extend.setPower(-0.3);
        myOpMode.sleep(200);

        while((myOpMode.opModeInInit() || myOpMode.opModeIsActive()) && readSensors() && (!liftIsHome || !extendIsHome)) {
            if (Math.abs(lastLiftEnc - liftEncoder) < 5) {
                lift.setPower(0);
                liftIsHome = true;
            }

            if (Math.abs(lastExtendEnc - extendEncoder) < 5) {
                extend.setPower(0);
                extendIsHome = true;
            }

            lastLiftEnc = liftEncoder;
            lastExtendEnc = extendEncoder;

            myOpMode.sleep(100);
            myOpMode.telemetry.addData("Arm", "LIS %s, EIH %s", liftIsHome, extendIsHome);
            myOpMode.telemetry.addData("Arm", "Homing");
            myOpMode.telemetry.update();
        }

        extend.setPower(0);
        lift.setPower(0.05);  // take up any slack
        myOpMode.sleep(200);
        lift.setPower(0.05);

        myOpMode.telemetry.addData("Arm", "Is Home");
        myOpMode.telemetry.update();

        // Reset encoders and determine lift and extend home positions.

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Globals.ARM_HAS_HOMED = true;
        wrist.setPosition(WRIST_HOME_ABS);
        setState(ManipulatorState.HOME);

        // reset any saved positions as they may be bogus
        lastBackExtend  = EXTEND_BACK_DISTANCE;
        lastFrontLift   = LIFT_FRONT_ANGLE;
        lastFrontExtend = EXTEND_FRONT_DISTANCE;
    }

    //------------ Servo Functions ------
    public void setWristOffset(int offset){
        wristOffset = (double)offset / 200.0;
        setRelativeWristAngle(WRIST_HOME_ABS);
    }

    public void setAbsoluteWristAngle( double angDeg) {
        wrist.setPosition((angDeg * WRIST_DEGREE_SCALE) + WRIST_POS_HOME + wristOffset);
    }

    public void setRelativeWristAngle( double angDeg) {
        setAbsoluteWristAngle(angDeg - liftAngle );
    }

    public void runManualGrippers() {
        if (myOpMode.gamepad1.left_trigger > 0.25) {
            openLeftGrabber();
        } else if (myOpMode.gamepad1.left_bumper) {
            closeLeftGrabber();
        }

        if (myOpMode.gamepad1.right_trigger > 0.25) {
            openRightGrabber();
        } else if (myOpMode.gamepad1.right_bumper){
            closeRightGrabber();
        }
    }

    public void wristToAutonomousPosition(){
        wristToHome();
        autoOpenGrabbers();
    }

    public void stackWackerUp(){
        stackWacker.setPosition(STACK_WACKER_UP);
    }

    public void stackWackerDown(){
        stackWacker.setPosition(STACK_WACKER_DOWN);
    }


    // ----------   LEFT Grabber functions.

    public void grabLeftPixel(){
        Globals.HOLDING_LEFT_PIXEL = true;
        closeLeftGrabber();
    }
    public void closeLeftGrabber(){
        clawL.setPosition(GRAB_LEFT_CLOSE);
    }

    public void dropLeftPixel(){
        Globals.HOLDING_LEFT_PIXEL = false;
        openLeftGrabber();
    }
    public void openLeftGrabber(){
        clawL.setPosition(GRAB_LEFT_OPEN);
    }

    // ----------   RIGHT Grabber functions.

    public void grabRightPixel(){
        Globals.HOLDING_RIGHT_PIXEL = true;
        closeRightGrabber();
    }
    public void closeRightGrabber(){
        clawR.setPosition(GRAB_RIGHT_CLOSE);
    }

    public void dropRightPixel(){
        Globals.HOLDING_RIGHT_PIXEL = false;
        openRightGrabber();
    }
    public void openRightGrabber(){
        clawR.setPosition(GRAB_RIGHT_OPEN);
    }

    public void dropPurplePixel() {
        if (Globals.PURPLE_PIXEL_ON_RIGHT) {
            dropRightPixel();
        } else {
            dropLeftPixel();
        }
    }

    public void dropYellowPixel() {
        if (Globals.PURPLE_PIXEL_ON_RIGHT) {
            dropLeftPixel();
        } else {
            dropRightPixel();
        }
    }

    public void dropPixels(){
        dropRightPixel();
        dropLeftPixel();
    }

    public void autoOpenGrabbers (){
        clawL.setPosition(GRAB_LEFT_AUTO);
        clawR.setPosition(GRAB_RIGHT_AUTO);
    }

    public void wristToHome(){
        setAbsoluteWristAngle(WRIST_HOME_ABS);
        Globals.WRIST_STATE = ManipulatorWristState.HOME;
    }

    public void wristToFrontScore(){
        setRelativeWristAngle(WRIST_SCORE_FRONT_REL);
        Globals.WRIST_STATE = ManipulatorWristState.FRONT_SCORE;
    }

    public void wristToPowerLift(){
        setAbsoluteWristAngle(WRIST_HANG_ABS);
        Globals.WRIST_STATE = ManipulatorWristState.BACK_SCORE;
    }

    public void wristToBackScore(){
        setAbsoluteWristAngle(WRIST_SCORE_BACK_ABS);
        Globals.WRIST_STATE = ManipulatorWristState.BACK_SCORE;
    }

    public boolean reverseGrippers(){
        return (currentState == BACK_SCORE);
    }

    //------------- state machine functions -------------

    private boolean smGotoHome       = false;
    private boolean smGotoSafeDriving = false;
    private boolean smGotoFrontScore  = false;
    private boolean smGotoBackScore  = false;
    private boolean smGotoPOWERLIFT  = false;

    public void gotoHome() {
        smGotoHome = true;
    }

    public void gotoSafeDriving() {
        smGotoSafeDriving = true;
    }

    public void gotoFrontScore() {
        if (currentState == ManipulatorState.FRONT_SCORE) {
            if (Globals.IS_AUTO) {
                setLiftSetpoint(Globals.PLACE_YELLOW_HIGH ? LIFT_HIGH_AUTO_ANGLE : LIFT_LOW_AUTO_ANGLE);
            } else {
                setLiftSetpoint(lastFrontLift);
            }
            setExtendSetpoint(lastFrontExtend);
            setAbsoluteWristAngle(60);
        } else {
            smGotoFrontScore = true;
        }
    }

    public void gotoBackScore() {
        if (currentState == ManipulatorState.BACK_SCORE) {
            setLiftSetpoint(LIFT_BACK_ANGLE);
            setExtendSetpoint(lastBackExtend);
            setAbsoluteWristAngle(60);
        } else {
            smGotoBackScore = true;
        }
    }

    public void gotoHang() {
        smGotoPOWERLIFT = true;
    }

    public boolean readyToHang() {

        return ((currentState == ManipulatorState.POWER_LIFTING) || (currentState == ManipulatorState.PL_EXTENDING));
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
            case H_RETRACTING:
//                if (extendLength < SAFE_EXTEND_DISTANCE) {  // was extendInPosition
                if (extendInPosition) {  // was extendInPosition
                    setLiftSetpoint(LIFT_HOME_ANGLE);
                    setState(ManipulatorState.H_OPEN);
                }
                break;

            case H_OPEN:
                // If either gripper is closed, wait for a sc to open them.
                smGotoSafeDriving = false;
                smGotoFrontScore  = false;
                smGotoBackScore  = false;
                smGotoPOWERLIFT = false;
                smGotoHome = false;
                setAbsoluteWristAngle(WRIST_HOME_ABS);
                setRangeEnable(true);
                if (!Globals.HOLDING_LEFT_PIXEL) {
                    openLeftGrabber();
                }
                if (!Globals.HOLDING_RIGHT_PIXEL) {
                    openRightGrabber();
                }
                setState(ManipulatorState.HOME);
                break;

            case HOME:
                setRangeEnable(true);
                if (smGotoHome) {
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setState(H_RETRACTING);
                } else if (smGotoSafeDriving) {
                    wristToBackScore();
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setState(ManipulatorState.SD_RETRACTING);
                } else if (smGotoFrontScore) {
                    if (Globals.IS_AUTO) {
                        setLiftSetpoint(Globals.PLACE_YELLOW_HIGH ? LIFT_HIGH_AUTO_ANGLE : LIFT_LOW_AUTO_ANGLE);
                    } else {
                        setLiftSetpoint(lastFrontLift);
                    }
                    wristToFrontScore();
                    setState(ManipulatorState.FS_LIFTING);
                } else if (smGotoBackScore) {
                    setLiftSetpoint(LIFT_BACK_ANGLE);
                    setStateWithDelay(ManipulatorState.BS_LIFTING, 0.25);
                } else if (smGotoPOWERLIFT) {
                    wristToPowerLift();
                    closeLeftGrabber();
                    closeRightGrabber();
                    setStateWithDelay(ManipulatorState.PL_ROTATE, 0.200);
                } else {
                    // setRelativeWristAngle(WRIST_HOME_ABS);
                }
                break;

            case SD_RETRACTING:
                if (extendInPosition) {
                    setLiftSetpoint(LIFT_HOME_ANGLE);
                    closeLeftGrabber();
                    closeRightGrabber();
                    setState(ManipulatorState.SAFE_DRIVING);
                }
                break;

            case SAFE_DRIVING:
                smGotoSafeDriving = false;
                if (smGotoHome) {
                    wristToHome();
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setState(H_RETRACTING);
                } else if (smGotoFrontScore) {
                    if (Globals.IS_AUTO) {
                        setLiftSetpoint(Globals.PLACE_YELLOW_HIGH ? LIFT_HIGH_AUTO_ANGLE : LIFT_LOW_AUTO_ANGLE);
                    } else {
                        setLiftSetpoint(lastFrontLift);
                    }
                    setState(ManipulatorState.FS_LIFTING);
                } else if (smGotoBackScore) {
                    setLiftSetpoint(LIFT_BACK_ANGLE);
                    setState(ManipulatorState.BS_LIFTING);
                }  else if (smGotoPOWERLIFT) {
                    wristToPowerLift();
                    setLiftSetpoint(LIFT_HANG_ANGLE);
                    setState(ManipulatorState.PL_LIFTING);
                }
                break;

            // -- Front Score  ----------
            case FS_LIFTING:
//              if (liftInPosition){
                if (liftAngle > SAFE_LIFT_ANGLE){
                    if (Globals.IS_AUTO) {
                        setExtendSetpoint(Globals.PLACE_YELLOW_HIGH ? EXTEND_HIGH_AUTO_DISTANCE : EXTEND_LOW_AUTO_DISTANCE);
                    } else {
                        setExtendSetpoint(lastFrontExtend);
                    }
                    setState(ManipulatorState.FRONT_SCORE);
                }
                break;

            case FRONT_SCORE:
                smGotoFrontScore = false;
                if (smGotoHome) {
                    wristToHome();
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setState(H_RETRACTING);
                } else if (smGotoSafeDriving) {
                    wristToBackScore();
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setStateWithDelay(ManipulatorState.SD_RETRACTING, 0.5);
                } else if (smGotoBackScore) {
                    setLiftSetpoint(LIFT_BACK_ANGLE);
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setState(ManipulatorState.BS_LIFTING);
                } else {
                    setRelativeWristAngle(WRIST_SCORE_FRONT_REL);
                }
                break;

            // -- Back Score  ----------
            case BS_LIFTING:
                wristToBackScore();
//                if (liftInPosition){
                if (liftAngle > 90){
//                    setStateWithDelay(ManipulatorState.BS_EXTEND, 0.25);
                    setState(ManipulatorState.BS_EXTEND);
                }
                break;

            case BS_EXTEND:
                setExtendSetpoint(lastBackExtend);
                setState(ManipulatorState.BACK_SCORE);
                break;

            case BACK_SCORE:
                smGotoBackScore = false;
                if (smGotoHome) {
                    setLiftSetpoint(SAFE_LIFT_ANGLE);
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setState(ManipulatorState.BACK_TO_HOME);
                } else if (smGotoSafeDriving) {
                    setLiftSetpoint(SAFE_LIFT_ANGLE);
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setState(ManipulatorState.BACK_TO_SAFE);
                }  else if (smGotoFrontScore) {
                    wristToBackScore();
                    setExtendSetpoint(lastFrontExtend);
                    setState(ManipulatorState.FS_LIFTING);
                }
                break;

            case BACK_TO_HOME:
                if (extendLength < SAFE_EXTEND_DISTANCE) {
                    wristToHome();
                    setLiftSetpoint(LIFT_HOME_ANGLE);
                    setState(ManipulatorState.H_OPEN);
                }
                break;

            case BACK_TO_SAFE:
                wristToBackScore();
                if (extendLength < SAFE_EXTEND_DISTANCE) {
                    setLiftSetpoint(LIFT_HOME_ANGLE);
                    setState(ManipulatorState.SAFE_DRIVING);
                }
                break;

            // -- Power Lifting --
            case PL_ROTATE:
                setLiftSetpoint(LIFT_HANG_ANGLE);
                setState(ManipulatorState.PL_LIFTING);
                break;

            case PL_LIFTING:
                smGotoPOWERLIFT = false;
                if (liftInPosition && (!myOpMode.gamepad2.left_stick_button && !myOpMode.gamepad2.right_stick_button)) {
                    setExtendSetpoint(EXTEND_LIFT_LENGTH);
                    setState(ManipulatorState.PL_EXTENDING);
                }  else if (smGotoHome) {
                    setLiftSetpoint(LIFT_HOME_ANGLE);
                    setState(ManipulatorState.H_OPEN);
                }
                break;

            case PL_EXTENDING:
                smGotoPOWERLIFT = false;
                if (extendInPosition) {
                    setState(ManipulatorState.POWER_LIFTING);
                } else if (smGotoHome) {
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setState(H_RETRACTING);
                }
                break;

            case POWER_LIFTING:
                smGotoPOWERLIFT = false;
                if (smGotoHome) {
                    setExtendSetpoint(EXTEND_HOME_DISTANCE);
                    setState(H_RETRACTING);
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
        Globals.ARM_STATE = newState;
        stateTimer.reset();
    }
}
