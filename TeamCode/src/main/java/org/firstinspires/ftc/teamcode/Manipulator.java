package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Manipulator {
    private static final double LIFT_GAIN       = 0.06;    // Strength of lift position control
    private static final double LIFT_ACCEL      = 1.5;      // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double LIFT_TOLERANCE  = 1.0;      // Controller is is "inPosition" if position error is < +/- this amount
    private static final double LIFT_MAX_AUTO   = 0.3;      // Maximum lift power

    private static final double EXTEND_GAIN     = 0.018;    // Strength of extend position control
    private static final double EXTEND_ACCEL    = 1.5;      // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double EXTEND_TOLERANCE = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double EXTEND_MAX_AUTO  = 0.3;      // Maximum extend power

    private static final double LIFT_COUNTS_PER_DEGREE = 11.05556 ; // 995 counts for 90 Deg
    private static final double EXTEND_COUNTS_PER_INCH = 158.944 ;  // 2861 counts for 18"
    private static final int    LIFT_HOME_OFFSET = (int)(5 * LIFT_COUNTS_PER_DEGREE);  // Home location is - 5 deg

    private static final double SHORT_HOLD_POWER = 0.17  ;
    private static final double LONG_HOLD_POWER  = 0.30  ;

    public  double liftAngle      = 0;   // Arm angle in degrees.  Horizontal = 0 degrees.  Increases to approximately 120 degrees.
    public  double extendLength   = 0;

    // Hardware interface Objects
    private DcMotor lift;       //  control the arm Lift Motor
    private DcMotor extend;     //  control the Linear Slide Extension Motor
    private Servo wrist;        //  control the claw rotation wrist
    private Servo clawL;        //  control the left claw open/close
    private Servo clawR;        //  control the right claw open/close

    private int liftEncoder    = 0;
    private int extendEncoder  = 0;

    private int liftEncoderHome   = 0;
    private int extendEncoderHome = 0;

    private double  liftSetpoint  = 0;
    private double  extendSetpoint   = 0;
    private boolean clawLClosed   = false;
    private boolean clawRClosed   = false;
    private double  wristAngle    = 0;

    private LinearOpMode myOpMode;
    private boolean showTelemetry = false;
    private ElapsedTime SMTimer   = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

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

        if (showTelemetry) {
            myOpMode.telemetry.addData("Arm Encoders L:X", "%6d %6d", liftEncoder, extendEncoder);
            myOpMode.telemetry.addData("Arm Pos L:X", "%5.1f %5.1f", liftAngle, extendLength);
        }
        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    public boolean runLiftControl() {
        double error = liftSetpoint - liftAngle;
        double errorPower = error * LIFT_GAIN;
        double anglePower = SHORT_HOLD_POWER  * Math.cos(Math.toRadians(liftAngle));
        double power = errorPower + anglePower;

        power = Range.clip(power, -0.4, 0.5);

        if (((power < 0) && (liftAngle < 25)) ||
            ((power > 0) && (liftAngle > 110))){
            power = 0;
        }

        lift.setPower(power);
        if (showTelemetry) {
            myOpMode.telemetry.addData("Arm E:P", "%6.1f %6.2f", error, power);
        }

        return (Math.abs(error) < LIFT_TOLERANCE);
    }

    public void setLiftSetpoint(double setpoint) {
        liftSetpoint = setpoint;
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

}
