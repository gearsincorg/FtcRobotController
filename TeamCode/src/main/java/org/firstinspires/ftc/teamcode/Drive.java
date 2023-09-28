package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

/*
 * This class is NOT an OpMode.  It is a Drive Class used by the RobotSimpleOdometry Opmode
 * All hardware interactions are handled here.
 *
 * This class manages the 4 motor drives, and the IMU
 * In addition, two Odometry Encoders are connected to the encoder inputs for two of the motors.
 * The motors and encoders are NOT directly related, they are just sharing motor channel assignments
 *
 */

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class Drive {

    // Adjust these numbers to suit your robot.
    private final double ODOM_COUNTS_PER_REV   = 8000.0 ;
    private final double ODOM_WHEEL_DIAM_INCH  = 2.0 ;
    private final double ODOM_INCHES_PER_COUNT =  (Math.PI * ODOM_WHEEL_DIAM_INCH) / ODOM_COUNTS_PER_REV;

    private final double ODOM_AXIAL_SCALE   = ODOM_INCHES_PER_COUNT;  // change to negative value if odom reads -ve moving forward
    private final double ODOM_LATERAL_SCALE = ODOM_INCHES_PER_COUNT;  // change to negative value if odom reads -ve moving left

    private final double POSITION_ACCURACY  = 0.5; // Required position accuracy
    private final double HEADING_ACCURACY   = 2.0; // Required heading accuracy

    private static final double AXIAL_GAIN   = 0.01;
    private static final double AXIAL_ACCEL  = 0.2;
    private static final double MAX_AXIAL    = 0.6;

    private static final double LATERAL_GAIN = 0.01;
    private static final double LATERAL_ACCEL= 0.2;
    private static final double MAX_LATERAL  = 0.6;

    private static final double YAW_GAIN     = 0.03;
    private static final double YAW_ACCEL    = 0.2;
    private static final double MAX_YAW      = 0.6;

    // Hardware interface Objects
    private DcMotor leftFrontDrive;     //  control the left front drive wheel
    private DcMotor rightFrontDrive;    //  control the right front drive wheel
    private DcMotor leftBackDrive;      //  control the left back drive wheel
    private DcMotor rightBackDrive;     //  control the right back drive wheel

    private DcMotor axialEncoder;       //  the Axial (front/back) Odometry Module
    private DcMotor lateralEncoder;     //  the Lateral (left/right) Odometry Module

    private LinearOpMode myOpMode;
    private IMU imu;

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
    private int rawLateralOdometer    = 0; // Unmodified lateral odometer count
    private int axialOdometerOffset   = 0; // Used to offset axial odometer
    private int lateralOdometerOffset = 0; // Used to offset lateral odometer

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU

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
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftfront_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightfront_drive");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftback_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightback_drive");
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        //  Connect to the encoder channels using the name of the motor on that channel.
        axialEncoder   = myOpMode.hardwareMap.get(DcMotor.class, "leftfront_drive");
        lateralEncoder = myOpMode.hardwareMap.get(DcMotor.class, "rightfront_drive");

        // Set the drive direction to ensure positive power drives each wheel forward.
        // Changes may be required depending on any gear reductions or 90 deg gear drives
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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
     *  Read all input devices to determine the robot's motion
     */
    public void readSensors() {
        rawAxialOdometer   = axialEncoder.getCurrentPosition();
        rawLateralOdometer = lateralEncoder.getCurrentPosition();
        axialDistance      = (rawAxialOdometer   - axialOdometerOffset) * ODOM_AXIAL_SCALE;
        lateralDistance    = (rawLateralOdometer - lateralOdometerOffset) * ODOM_LATERAL_SCALE;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        heading =  orientation.getYaw(AngleUnit.DEGREES);
        turnRate = angularVelocity.zRotationRate;
    }

    /**
     * Drive in the axial (forward/reverse) direction and maintain the current heading and lateral position
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     */
    public void driveAxial(double distanceInches, double power) {
        //  Ensure that power is positive
        power = Math.abs(power);
        axialController.reset(distanceInches);
        lateralController.reset(0);

        resetOdometry();
        while (Math.abs(axialDistance - distanceInches) > POSITION_ACCURACY){
            // calculate axial power to reach desired distance
            // calculate lateral power to correct any lateral drift
            double axialPower   = axialController.getOutput(axialDistance);
            double lateralPower = lateralController.getOutput(lateralDistance);
            double yawPower     = yawController.getOutput(heading);

            // implement desired axis powers
            moveRobot(axialPower, lateralPower, yawPower);
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
        axialController.reset(0);
        lateralController.reset(distanceInches);

        resetOdometry();
        while (Math.abs(lateralDistance - distanceInches) > POSITION_ACCURACY){
            // calculate axial power to correct any axial drift.  Limit max value
            // calculate lateral power to reach desired distance
            double axialPower   = axialController.getOutput(axialDistance);
            double lateralPower = lateralController.getOutput(lateralDistance);
            double yawPower     = yawController.getOutput(heading);

            // implement desired axis powers
            moveRobot(axialPower, lateralPower, yawPower);
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     */
    public void turnToHeading(double headingDeg, double power) {
        yawController.reset(headingDeg);

        while (Math.abs(normalizeHeading(headingDeg - heading)) > HEADING_ACCURACY) {

            // calculate yaw power to obtain desire heading
            double yawPower     = yawController.getOutput(heading);

            // implement desired axis powers
            moveRobot(0, 0, yawPower);
            myOpMode.sleep(10);
        }
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
}

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

    public ProportionalControl(double gain, double accelLimit, double outputLimit, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.outputLimit = outputLimit;
        this.circular = circular;
        lastOutput = 0.0;
        setPoint = 0.0;
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        double error = setPoint - input;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

        // calculate output power using gain and other limits.
        double output = (error * gain);

        if ((output - lastOutput) > accelLimit) {
            output = lastOutput + accelLimit;
        } else if ((output - lastOutput) < -accelLimit) {
            output = lastOutput - accelLimit;
        }

        output = Range.clip(output, -outputLimit, outputLimit);
        lastOutput = output;
        return output;
    }

    // Saves a new setpoint and resets the output power history.
    public void reset(double setPoint) {
        this.setPoint = setPoint;
        lastOutput = 0.0;
    }
}
