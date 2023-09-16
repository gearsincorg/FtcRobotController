/* Copyright (c) 2019 G-FORCE.
 * This class is used to define all the specific Hardware elements for the G-FORCE robot
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {
    public static enum AllianceColor {
        UNKNOWN_COLOR,
        RED,
        BLUE
    }

    public static final String TAG = "Hardware";
    public static final boolean LOGGING = false;  // Set to true to add data to logfile


    /* Public OpMode members. */
    public AllianceColor allianceColor = AllianceColor.UNKNOWN_COLOR;
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx leftBackDrive = null;
    public DcMotorEx rightBackDrive = null;

    public IMU imu = null;

    public final double ACCELERATION_LIMIT  = 1000;  // MM per second per second  was 1524

    public final double YAW_GAIN            = 0.010;  // Rate at which we respond to heading error 0.013
    public final double LATERAL_GAIN        = 0.0025; // Distance from x axis that we start to slow down. 0027
    public final double AXIAL_GAIN          = 0.0015; // Distance from target that we start to slow down. 0017

    public final double MAX_VELOCITY        = 2500;  // Counts per second
    public final double MAX_VELOCITY_MMPS   = 2540;  // MM Per Second
    public final double AUTO_ROTATION_DPS   = 2540;   // Degrees per second

    public double headingSetpoint = 0;

    public int  encoderLF = 0;
    public int  encoderRF = 0;
    public int  encoderLB = 0;
    public int  encoderRB = 0;

    private int startLeftBack = 0;
    private int startLeftFront = 0;
    private int startRightBack = 0;
    private int startRightFront = 0;
    private int deltaLeftBack = 0;
    private int deltaLeftFront = 0;
    private int deltaRightBack = 0;
    private int deltaRightFront = 0;

    // Driving constants Yaw heading
    final double HEADING_GAIN       = 0.015;  // Was 0.012
    final double TURN_RATE_TC       = 0.6;
    final double STOP_TURNRATE      = 0.020;

    final double YAW_IS_CLOSE = 2.0;  // angle within which we are "close"

    final double AXIAL_ENCODER_COUNTS_PER_MM   = 0.958; // was 0.8602
    final double LATERAL_ENCODER_COUNTS_PER_MM = 0.958;  // was 0.9134

    // Robot states that we share with others
    public double axialMotion = 0;
    public double lateralMotion = 0;
    public double currentHeading = 0;

    private double filteredTurnRate = 0;
    private double driveAxial = 0;
    private double driveYaw = 0;
    private double driveLateral = 0;
    private double startTime = 0;

    private static LinearOpMode myOpMode = null;

    // Sensor Read Info.
    /* local OpMode members. */
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime navTime = new ElapsedTime();

    /* Constructor */
    public RobotHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode) {
        // Save reference to Hardware map
        myOpMode = opMode;

        // Define and Initialize Motors
        leftBackDrive = configureMotor("left_back_drive", DcMotor.Direction.REVERSE);
        leftFrontDrive = configureMotor("left_front_drive", DcMotor.Direction.REVERSE);
        rightBackDrive = configureMotor("right_back_drive", DcMotor.Direction.FORWARD);
        rightFrontDrive = configureMotor("right_front_drive", DcMotor.Direction.FORWARD);

        resetEncoders();


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        //Set the mode of the motors
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Configure a motor
    public DcMotorEx configureMotor( String name, DcMotor.Direction direction) {
        DcMotorEx motorObj = myOpMode.hardwareMap.get(DcMotorEx.class, name);
        motorObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorObj.setDirection(direction);
        motorObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motorObj;
    }

    // Autonomous driving methods
    /**
     * Drive a set distance at a set heading at a set speed until the timeout occurs
     *
     * @param mm
     * @param heading
     * @param vel
     * @param timeOutSec
     * @return
     */
    public boolean driveAxialVelocity(double mm, double heading, double vel, double timeOutSec, boolean flipOnBlue) {
        return (true);
    }

    /**
     * Drive a set distance at a set heading at a set speed until the timeout occurs
     *
     * @param mm
     * @param heading
     * @param vel
     * @param timeOutSec
     * @return
     */
    public boolean driveLateralVelocity(double mm, double heading, double vel, double timeOutSec, boolean flipOnBlue) {
        return (true);
    }

    //
    public double getProfileVelocity(double topVel, double dTraveled, double dGoal) {
        return (0.0);
    }

    // Common location to read all sensors
    public void readSensors() {

        // If we are exiting, don't read bulk data
        if (!myOpMode.isStopRequested()) {

            encoderLF =         leftFrontDrive.getCurrentPosition();
            encoderRF =         rightFrontDrive.getCurrentPosition();
            encoderLB =         leftBackDrive.getCurrentPosition();
            encoderRB =         rightBackDrive.getCurrentPosition();
        }
    }

    public double getAxialMotion() {
        // NOTE:  Must call updateMotion() once before calling this method;
        return (axialMotion);
    }

    public double getLateralMotion() {
        // NOTE:  Must call updateMotion() once before calling this method;
        return (lateralMotion);
    }

    public void startMotion() {
        readSensors();
        startLeftBack = encoderLB;
        startLeftFront = encoderLF;
        startRightBack = encoderRB;
        startRightFront = encoderRF;
        deltaLeftBack = 0;
        deltaLeftFront = 0;
        deltaRightBack = 0;
        deltaRightFront = 0;
        startTime = runTime.time();
    }

    public boolean updateMotion() {
        readSensors();
        deltaLeftBack = encoderLB - startLeftBack;
        deltaLeftFront = encoderLF - startLeftFront;
        deltaRightBack = encoderRB - startRightBack;
        deltaRightFront = encoderRF - startRightFront;
        axialMotion = ((deltaLeftBack + deltaLeftFront + deltaRightBack + deltaRightFront) / 4);
        axialMotion /= AXIAL_ENCODER_COUNTS_PER_MM;
        lateralMotion = ((-deltaLeftBack + deltaLeftFront + deltaRightBack - deltaRightFront) / 4);
        lateralMotion /= LATERAL_ENCODER_COUNTS_PER_MM;
        return (true);
    }

    public double getMotionTime() {
        return(runTime.time() - startTime);
    }


    public void resetEncoders() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void showEncoders() {
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    public boolean notTurning() {

        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        double rate = angularVelocity.zRotationRate;

        filteredTurnRate += ((rate - filteredTurnRate) * TURN_RATE_TC);
        myOpMode.telemetry.addData("Turn Rate", "%6.3f", filteredTurnRate);

        return (Math.abs(filteredTurnRate) < STOP_TURNRATE);
    }

    // Robot movement using +/- MAX_VELOCITY
    public void setAxialVelocity(double axialV) {
        driveAxial = Range.clip(axialV * AXIAL_ENCODER_COUNTS_PER_MM, -MAX_VELOCITY, MAX_VELOCITY);
    }

    public void setYawVelocity(double yawV) {
        driveYaw = Range.clip(yawV * AXIAL_ENCODER_COUNTS_PER_MM, -MAX_VELOCITY, MAX_VELOCITY);
    }

    public void setLateralVelocity(double lateralV) {
        driveLateral = Range.clip(lateralV * LATERAL_ENCODER_COUNTS_PER_MM, -MAX_VELOCITY, MAX_VELOCITY);
    }

    public void moveRobotVelocity(double axialV, double yawV, double lateralV) {
        setAxialVelocity(axialV);
        setYawVelocity(yawV);
        setLateralVelocity(lateralV);
        moveRobotVelocity();
    }

    public void moveRobotVelocity() {
        //calculate required motor speeds
        double leftFrontVel = driveAxial - driveYaw + driveLateral;
        double leftBackVel = driveAxial - driveYaw - driveLateral;
        double rightFrontVel = driveAxial + driveYaw - driveLateral;
        double rightBackVel = driveAxial + driveYaw + driveLateral;

        double biggest = Math.max(Math.abs(leftFrontVel), Math.abs(rightFrontVel));
        biggest = Math.max(biggest, Math.abs(leftBackVel));
        biggest = Math.max(biggest, Math.abs(rightBackVel));

        if (biggest > MAX_VELOCITY) {
            double scale = MAX_VELOCITY / biggest;
            leftFrontVel *= scale;
            rightFrontVel *= scale;
            leftBackVel *= scale;
            rightBackVel *= scale;
        }

        leftFrontDrive.setVelocity(leftFrontVel);
        rightFrontDrive.setVelocity(rightFrontVel);
        leftBackDrive.setVelocity(leftBackVel);
        rightBackDrive.setVelocity(rightBackVel);

        // Log.d("G-FORCE AUTO", String.format("M %5.1f %5.1f %5.1f %5.1f ", leftFrontVel, rightFrontVel, leftBackVel, rightBackVel));
    }


    // NOTE: updateMotion() or getHeading() MUST be called prior to this call
    public boolean setYawVelocityToHoldHeading(double newSetpoint) {

        setHeadingSetpoint(newSetpoint);
        return (setYawVelocityToHoldHeading());
    }

    // NOTE: updateMotion() or getHeading() MUST be called prior to this call
    public boolean setYawVelocityToHoldHeading() {
        // double error = normalizeHeading(headingSetpoint - getHeading());
        double error = normalizeHeading(headingSetpoint - currentHeading);
        double yaw = Range.clip(error * HEADING_GAIN, -0.25, 0.25);


        setYawVelocity(yaw * AUTO_ROTATION_DPS);
        return (Math.abs(error) < YAW_IS_CLOSE);
    }

    public void setAxialPower (double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    public void stopRobot() {
        moveRobotVelocity(0, 0, 0);
    }

    public double getHeading() {

        Orientation angles;
        double heading;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        heading = orientation.getYaw(AngleUnit.DEGREES);

        currentHeading = heading;
        return (currentHeading);
    }

    public double normalizeHeading(double heading) {
        while (heading <= -180) {
            heading += 360;
        }
        while (heading >= 180) {
            heading -= 360;
        }
        return heading;
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void setHeadingSetpoint(double newSetpoint) {
        headingSetpoint = newSetpoint;
    }
}
