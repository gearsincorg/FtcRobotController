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

    // Public Variables


    // Adjust these numbers to suit your robot.
    private final double ODOM_COUNTS_PER_REV   = 8000.0 ;
    private final double ODOM_WHEEL_DIAM_INCH  = 2.0 ;
    private final double ODOM_INCHES_PER_COUNT =  (Math.PI * ODOM_WHEEL_DIAM_INCH) / ODOM_COUNTS_PER_REV;

    private final double ODOM_AXIAL_SCALE   = ODOM_INCHES_PER_COUNT;  // change to negative value if odom reads -ve forward
    private final double ODOM_LATERAL_SCALE = ODOM_INCHES_PER_COUNT;  // change to negative value if odom reads -ve left

    private final double POSITION_ACCURACY  = 0.5; // this is how close to the desired position the robot must get before moving on.
    private final double HEADING_ACCURACY   = 2.0; // this is how close to the desired heading the robot must get before moving on.

    private static final double AXIAL_GAIN   = 0.01;
    private static final double MAX_AXIAL    = 0.6;

    private static final double LATERAL_GAIN = 0.01;
    private static final double MAX_LATERAL  = 0.6;

    private static final double HEADING_GAIN = 0.01;
    private static final double MAX_YAW      = 0.6;

    // Hardware interface Objects
    private DcMotor leftFrontDrive;     //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive;    //  Used to control the right front drive wheel
    private DcMotor leftBackDrive;      //  Used to control the left back drive wheel
    private DcMotor rightBackDrive;     //  Used to control the right back drive wheel

    private DcMotor axialEncoder;       //  Used to read the Axial (front/back) Odometry Module
    private DcMotor lateralEncoder;     //  Used to read the Lateral (left/right) Odometry Module

    private LinearOpMode myOpMode;
    private IMU imu;

    // Public Variables
    public double axialDistance      = 0; // scaled axial distance (+ = forward)
    public double lateralDistance    = 0; // scaled lateral distance (+ = left)

    // Private Variables
    private int rawAxialOdometer      = 0; // Unmodified axial odometer count
    private int rawLateralOdometer    = 0; // Unmodified lateral odometer count
    private int axialOdometerOffset   = 0; // Used to offset axial odometer
    private int lateralOdometerOffset = 0; // Used to offset lateral odometer

    private double heading            = 0; // Latest Robot heading from IMU
    private double turnRate           = 0; // Latest Robot Turn Rate from IMU
    private double headingSetPoint    = 0; // Robot tries to hold this heading.

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

    public void driveAxial(double distanceInches, double power) {
        //  Ensure that power is positive
        power = Math.abs(power);

        resetOdometry();
        while (Math.abs(axialDistance - distanceInches) > POSITION_ACCURACY){
            // calculate axial power to reach desired distance, Limit max value
            // calculate lateral power to correct any lateral drift, Limit max value
            double axialPower   = Range.clip((distanceInches - axialDistance) * AXIAL_GAIN, -MAX_AXIAL, MAX_AXIAL);
            double lateralPower = Range.clip(-lateralDistance * LATERAL_GAIN, -MAX_LATERAL, MAX_LATERAL);

            // implement desired axis powers
            moveRobot(axialPower, lateralPower, headingCorrectionPower());
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    public void driveLateral(double distanceInches, double power) {
        //  Ensure that power is positive
        power = Math.abs(power);

        resetOdometry();
        while (Math.abs(lateralDistance - distanceInches) > POSITION_ACCURACY){
            // calculate axial power to correct and axial drift.  Limit max value
            // calculate lateral power to correct any lateral drift, Limit max value
            double axialPower = Range.clip(-axialDistance * AXIAL_GAIN, -MAX_AXIAL, MAX_AXIAL);
            double lateralPower = Range.clip((distanceInches - lateralDistance) * LATERAL_GAIN, -MAX_LATERAL, MAX_LATERAL);

            // implement desired axis powers
            moveRobot(axialPower, lateralPower, headingCorrectionPower());
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    public double headingCorrectionPower(){
        double error = normalizeHeading(headingSetPoint - heading);
        return Range.clip((error * HEADING_GAIN), -MAX_YAW, MAX_YAW);
    }

    public void turnToHeading(double headingDeg, double power) {
        while (Math.abs(normalizeHeading(headingDeg - heading)) > HEADING_ACCURACY) {

            // implement desired axis powers
            moveRobot(0, 0, headingCorrectionPower());
            myOpMode.sleep(10);
        }
    }

    public double normalizeHeading(double heading) {

        // Normalize the error to be within +/- 180 degrees
        while (heading > 180)  heading -= 360;
        while (heading <= -180) heading += 360;

        return heading;
    }

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

    public void stopRobot() {
        moveRobot(0,0,0);
    }

    public void resetOdometry() {
        // reset odometry encoders
        readSensors();
        axialOdometerOffset = rawAxialOdometer;
        lateralOdometerOffset = rawLateralOdometer;
        axialDistance = 0.0;
        lateralDistance = 0.0;
    }
}
