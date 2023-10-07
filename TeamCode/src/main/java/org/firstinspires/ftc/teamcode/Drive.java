package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class Drive {

    // Adjust these numbers to suit your robot.
    private final double ODOM_COUNTS_PER_REV   = 8000.0 ;
    private final double ODOM_WHEEL_DIAM_INCH  = 2.35 ;
    private final double ODOM_INCHES_PER_COUNT =  (Math.PI * ODOM_WHEEL_DIAM_INCH) / ODOM_COUNTS_PER_REV;

    private final double ODOM_AXIAL_SCALE   =  ODOM_INCHES_PER_COUNT;  // change to negative value if odom reads -ve moving forward
    private final double ODOM_LATERAL_SCALE = -ODOM_INCHES_PER_COUNT;  // change to negative value if odom reads -ve moving left

    private final double POSITION_ACCURACY  = 0.25; // Required position accuracy (+/- inches)
    private final double HEADING_ACCURACY   = 2.0; // Required heading accuracy

    private static final double AXIAL_GAIN   = 0.025;
    private static final double AXIAL_ACCEL  = 1.5;
    private static final double MAX_AXIAL    = 0.6;

    private static final double LATERAL_GAIN = 0.025;
    private static final double LATERAL_ACCEL= 1.5;
    private static final double MAX_LATERAL  = 0.6;

    private static final double YAW_GAIN     = 0.03;
    private static final double YAW_ACCEL    = 1.5;
    private static final double MAX_YAW      = 0.6;

    // Hardware interface Objects
    private DcMotor leftFrontDrive;     //  control the left front drive wheel
    private DcMotor rightFrontDrive;    //  control the right front drive wheel
    private DcMotor leftBackDrive;      //  control the left back drive wheel
    private DcMotor rightBackDrive;     //  control the right back drive wheel

    private DcMotor axialEncoder;       //  the Axial (front/back) Odometry Module (may overlap with motor, or may not)
    private DcMotor lateralEncoder;     //  the Lateral (left/right) Odometry Module (may overlap with motor, or may not)

    private LinearOpMode myOpMode;
    private IMU imu;
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    // Establish a proportional controller for each axis to determine the required power to achieve a setpoint.
    private ProportionalControl axialController     = new ProportionalControl(AXIAL_GAIN, AXIAL_ACCEL, MAX_AXIAL, false);
    private ProportionalControl lateralController   = new ProportionalControl(LATERAL_GAIN, LATERAL_ACCEL, MAX_LATERAL, false);
    private ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, MAX_YAW, false);

    // Public Variables
    public double axialDistance      = 0; // scaled axial distance (+ = forward)
    public double lateralDistance    = 0; // scaled lateral distance (+ = left)
    public double heading            = 0; // Latest Robot heading from IMU

    // Private Variables
    private int rawAxialOdometer      = 0; // Unmodified axial odometer count
    private int axialOdometerOffset   = 0; // Used to offset axial odometer
    private int rawLateralOdometer    = 0; // Unmodified lateral odometer count
    private int lateralOdometerOffset = 0; // Used to offset lateral odometer
    private double rawHeading         = 0; // Unmodified heading (degrees)
    private double headingOffset      = 0; // Used to offset heading

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU
    private boolean showTelemetry     = false;

    // Drive Constructor
    public Drive (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Drive Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     */
    public void init()
    {
        // Initialize the hardware variables. Note that the strings used to 'get' each
        // device must match the names assigned during the robot configuration.
        // Set the drive direction to ensure positive power drives each wheel forward.
        leftFrontDrive  = setupDriveMotor("leftfront_drive", DcMotor.Direction.REVERSE);
        rightFrontDrive = setupDriveMotor("rightfront_drive", DcMotor.Direction.FORWARD);
        leftBackDrive  = setupDriveMotor( "leftback_drive", DcMotor.Direction.REVERSE);
        rightBackDrive = setupDriveMotor( "rightback_drive",DcMotor.Direction.FORWARD);
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        //  Connect to the encoder channels using the name of that channel.
        axialEncoder   = myOpMode.hardwareMap.get(DcMotor.class, "leftfront_drive");
        lateralEncoder = myOpMode.hardwareMap.get(DcMotor.class, "rightfront_drive");

        // Set all hubs to use the AUTO Bulk Caching mode for encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Tell the software how the Control Hub is mounted on the robot to align the XYZ axes correctly
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                             RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    /**
     *   Setup a drive motor with passed parameters.  Ensure encoder is reset.
     * @param deviceName  Text name associated with motor in Robot Configuration
     * @param direction   Desired direction to make the wheel runFORWARD with positive input
     * @return
     */
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return aMotor;
    }

    /**
     *  Read all input devices to determine the robot's motion
     *  always return true so this can be used in while loop conditions
     */
    public boolean readSensors() {
        rawAxialOdometer   = axialEncoder.getCurrentPosition();
        rawLateralOdometer = lateralEncoder.getCurrentPosition();
        axialDistance      = (rawAxialOdometer   - axialOdometerOffset) * ODOM_AXIAL_SCALE;
        lateralDistance    = (rawLateralOdometer - lateralOdometerOffset) * ODOM_LATERAL_SCALE;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        rawHeading = orientation.getYaw(AngleUnit.DEGREES);
        heading    = rawHeading - headingOffset;

        turnRate = angularVelocity.zRotationRate;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Odom A:L", "%6d %6d", rawAxialOdometer, rawLateralOdometer);
            myOpMode.telemetry.addData("Distance A:L", "%5.1f %5.1f", axialDistance, lateralDistance);
            myOpMode.telemetry.addData("Heading D:R", "%5.0f %5.1f", heading, turnRate);
        }
        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction and maintain the current heading and lateral position
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     */
    public void driveAxial(double distanceInches, double power) {
        //  Ensure that power is positive
        power = Math.abs(power);
        resetOdometry();
        axialController.reset(distanceInches);
        lateralController.reset(0);
        yawController.reset();

        while (myOpMode.opModeIsActive() && readSensors() && (Math.abs(axialDistance - distanceInches) > POSITION_ACCURACY)){
            runPositionControl();
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Drive in the lateral (left/right strafe) direction and maintain the current heading and axial position
     * @param distanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     */
    public void driveLateral(double distanceInches, double power) {
        //  Ensure that power is positive
        power = Math.abs(power);
        resetOdometry();
        axialController.reset(0);
        lateralController.reset(distanceInches);
        yawController.reset();

        while (myOpMode.opModeIsActive() && readSensors() && (Math.abs(lateralDistance - distanceInches) > POSITION_ACCURACY)){

            runPositionControl();
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     */
    public void turnToHeading(double headingDeg, double power) {

        yawController.reset(headingDeg);
        while (myOpMode.opModeIsActive() && readSensors() && (Math.abs(normalizeHeading(headingDeg - heading)) > HEADING_ACCURACY)) {
            runHeadingControl();
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Hold the last position setpoint.  Used to allow robot time to stabilize;
     * @param holdSeconds time to hold current heading/position.
     */
    public void holdLastPosition(double holdSeconds) {

        holdTimer.reset();
        axialController.reset();
        lateralController.reset();
        while (myOpMode.opModeIsActive() && readSensors() && (holdTimer.time() < holdSeconds)) {
            runPositionControl();
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Hold the last heading setpoint.   Used to allow robot time to stabilize;
     * @param holdSeconds time to hold current heading/position.
     */
    public void holdLastHeading(double holdSeconds) {

        holdTimer.reset();
        yawController.reset();
        while (myOpMode.opModeIsActive() && readSensors() && (holdTimer.time() < holdSeconds)) {
            runHeadingControl();
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    //  ########################  Low level control functions.  ###############################

    /**
     * Low level call to implement position/heading control
     */
    public void runPositionControl(){
        // calculate yaw power to obtain desire heading
        double axialPower   = axialController.getOutput(axialDistance);
        double lateralPower = lateralController.getOutput(lateralDistance);
        double yawPower     = yawController.getOutput(heading);

        // implement desired axis powers
        // implement desired axis powers
        moveRobot(axialPower, lateralPower, yawPower);
    }

    /**
     * Low level call to implement just heading control
     */
    public void runHeadingControl() {
        // calculate yaw power to obtain desire heading
        double yawPower     = yawController.getOutput(heading);

        // implement desired axis powers
        moveRobot(0, 0, yawPower);
    }

    /**
     * Normalize the heading to be within +/- 180 degrees
     * @param heading  desired heading in degrees.
     * @return
     */
    public double normalizeHeading(double heading) {
        while (heading > 180)  heading -= 360;
        while (heading <= -180) heading += 360;
        return heading;
    }

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * @param axial
     * @param lateral
     * @param yaw
     */
    public void moveRobot(double axial, double lateral, double yaw){

        double lF = axial - lateral - yaw;
        double rF = axial + lateral + yaw;
        double lB = axial + lateral - yaw;
        double rB = axial - lateral + yaw;

        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

        //normalize the motor values
        if (max > 1.0)  {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }

        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

        if (showTelemetry) {
            myOpMode.telemetry.addData("Axes A:L:Y", "%5.2f %5.2f %5.2f", axial, lateral, yaw);
            myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
            myOpMode.telemetry.update(); //  Assume this is the last thing done in the loop.
        }
    }

    /**
     * Stop all motors.
     */
    public void stopRobot() {
        moveRobot(0,0,0);
    }

    /**
     * Set odometry positions to zero.
     */
    public void resetOdometry() {
        readSensors();
        axialOdometerOffset = rawAxialOdometer;
        lateralOdometerOffset = rawLateralOdometer;
        axialDistance = 0.0;
        lateralDistance = 0.0;
    }

    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        heading = 0;
    }

    /**
     * Set drive, and controller telemetry on or off
     */
    public void showTelemetry(boolean show){
        showTelemetry = show;
    }
}

//****************************************************************************************************
//****************************************************************************************************

/***
 * This class is used to implement a proportional controller which can calculate the desired output power
 * to get an axis to the desired setpoint value.
 * It also implements an acceleration limit
 */
class ProportionalControl {
    double  lastOutput;
    double  gain;
    double  accelLimit;
    double  outputLimit;
    double  setPoint;
    boolean circular;
    ElapsedTime cycleTime = new ElapsedTime();

    public ProportionalControl(double gain, double accelLimit, double outputLimit, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.outputLimit = outputLimit;
        this.circular = circular;
        reset(0.0);
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

        // calculate output power using gain and other limits.
        double output = (error * gain);

        if ((output - lastOutput) > dV) {
            output = lastOutput + dV;
        } else if ((output - lastOutput) < -dV) {
            output = lastOutput - dV;
        }

        output = Range.clip(output, -outputLimit, outputLimit);
        lastOutput = output;
        cycleTime.reset();
        return output;
    }

    // Saves a new setpoint and resets the output power history.
    public void reset(double setPoint) {
        this.setPoint = setPoint;
        reset();
    }

    // Just restart the accel timer and output to 0
    public void reset() {
        cycleTime.reset();
        lastOutput = 0.0;
    }
}
