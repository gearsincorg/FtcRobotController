package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Manipulator {
    private static final double LIFT_GAIN       = 0.018;    // Strength of lift position control
    private static final double LIFT_ACCEL      = 1.5;      // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double LIFT_TOLERANCE  = 1.0;      // Controller is is "inPosition" if position error is < +/- this amount
    private static final double LIFT_AUTO_YAW   = 0.3;      // Maximum lift power

    private static final double EXTEND_GAIN     = 0.018;    // Strength of extend position control
    private static final double EXTEND_ACCEL    = 1.5;      // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double EXTEND_TOLERANCE = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double EXTEND_AUTO_YAW = 0.3;      // Maximum extend power

    private static final double LIFT_COUNTS_PER_DEGREE = 11.05556 ; // 995 counts for 90 Deg
    private static final double EXTEND_COUNTS_PER_INCH = 158.944 ;  // 2861 counts for 18"
    private static final int    LIFT_HOME_OFFSET = (int)(5 * LIFT_COUNTS_PER_DEGREE);  // Home location is - 5 deg


    // Hardware interface Objects
    private DcMotor lift;       //  control the arm Lift Motor
    private DcMotor extend;     //  control the Linear Slide Extension Motor
    private Servo wrist;        //  control the claw rotation wrist
    private Servo clawL;        //  control the left claw open/close
    private Servo clawR;        //  control the right claw open/close

    private int rawLiftEncoder    = 0;
    private int rawExtendEncoder  = 0;
    private double liftAngle      = 0;   // Arm angle in degrees.  Horizontal = 0 degrees.  Increases to approximately 120 degrees.
    private double extendLength   = 0;

    private int liftEncoderHome   = 0;
    private int extendEncoderHome = 0;

    private double  listSetpoint  = 0;
    private double  armSetpoint   = 0;
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
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Requires motor encoder cables to be hooked up.

        extend = myOpMode.hardwareMap.get(DcMotor.class, "extend");
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Requires motor encoder cables to be hooked up.

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
        rawLiftEncoder    = lift.getCurrentPosition() ;
        rawExtendEncoder  = extend.getCurrentPosition() ;
        int liftEncoder    = rawLiftEncoder   - liftEncoderHome;
        int extendEncoder  = rawExtendEncoder - extendEncoderHome;

        liftAngle = liftEncoder / LIFT_COUNTS_PER_DEGREE;
        extendLength = extendEncoder / EXTEND_COUNTS_PER_INCH;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Arm Encoders L:X", "%6d %6d", liftEncoder, extendEncoder);
            myOpMode.telemetry.addData("Arm Pos L:X", "%5.1f %5.1f", liftAngle, extendLength);
        }
        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    public void homeArm() {
        int lastLiftPos   = rawLiftEncoder;
        int lastExtendPos = rawExtendEncoder;
        boolean liftIsHome = false;
        boolean extendIsHome = false;

        // Start retracting arm
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(-0.1);
        extend.setPower(-0.1);
        myOpMode.sleep(200);
        while(myOpMode.opModeIsActive() && readSensors() && (!liftIsHome || !extendIsHome)) {
            if (Math.abs(lastLiftPos - rawLiftEncoder) < 5) {
                lift.setPower(0);
                liftIsHome = true;
            }

            if (Math.abs(lastExtendPos - rawExtendEncoder) < 5) {
                extend.setPower(0);
                extendIsHome = true;
            }

            lastLiftPos = rawLiftEncoder;
            lastExtendPos = rawExtendEncoder;
            myOpMode.sleep(100);
            myOpMode.telemetry.addData("Arm", "Homing");
            myOpMode.telemetry.update();
        }
        lift.setPower(0);
        extend.setPower(0);
        myOpMode.telemetry.addData("Arm", "Is Home");
        myOpMode.telemetry.update();

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset the home positions.  Add any offsets here.
        liftEncoderHome = rawLiftEncoder + LIFT_HOME_OFFSET;
        extendEncoderHome = rawExtendEncoder;
    }

    public void update(){
        if (showTelemetry) {
        }
    }

}
