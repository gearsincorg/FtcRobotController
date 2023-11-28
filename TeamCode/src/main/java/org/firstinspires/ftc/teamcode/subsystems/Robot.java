/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class Robot {
    // Adjust these numbers to suit your robot.
    private final double ODOM_INCHES_PER_COUNT      = 0.002969;   //  GoBilda Odometry Pod (1/226.8)
    private final boolean INVERT_DRIVE_ODOMETRY     = true;    //  When driving FORWARD, the odometry value MUST increase.  If it does not, flip the value of this constant.
    private final boolean INVERT_STRAFE_ODOMETRY    = true;    //  When strafing to the LEFT, the odometry value MUST increase.  If it does not, flip the value of this constant.

    private static final double DRIVE_GAIN          = 0.03;    // Strength of axial position control
    private static final double DRIVE_ACCEL         = 2.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO      = 0.6;     // "default" Maximum Axial power limit during autonomous

    private static final double STRAFE_GAIN         = 0.03;    // Strength of lateral position control
    private static final double STRAFE_ACCEL        = 1.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double STRAFE_TOLERANCE    = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double STRAFE_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double STRAFE_MAX_AUTO     = 0.6;     // "default" Maximum Lateral power limit during autonomous

    private static final double YAW_GAIN            = 0.015;   // Strength of Yaw position control
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    // Adjust these numbers to suit your robot.
    private final double V_DESIRED_DISTANCE         = 7.0;     //  this is how close the camera should get to the target (inches)
    private final double V_DRIVE_TOLERANCE          = 0.4;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    private final double V_DRIVE_GAIN  =  0.02  ;   //  Forward Speed Control "Gain".
    private final double V_STRAFE_GAIN =  0.05 ;    //  Strafe Speed Control "Gain".
    private final double V_STRAFE_MAX_AUTO = 0.5;   // "default" Maximum Lateral power limit during autonomous

    // Public Members
    public double driveDistance     = 0; // scaled axial distance (+ = forward)
    public double strafeDistance    = 0; // scaled lateral distance (+ = left)
    public double heading           = 0; // Latest Robot heading from IMU
    public double pitch             = 0; // Latest Robot tilt up from IMU

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public ProportionalControl driveController     = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public ProportionalControl strafeController    = new ProportionalControl(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    public ProportionalControl v_driveController   = new ProportionalControl(V_DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, V_DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public ProportionalControl v_strafeController  = new ProportionalControl(V_STRAFE_GAIN, STRAFE_ACCEL, V_STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);

    // ---  Private Members

    // Hardware interface Objects
    private DcMotor leftFrontDrive;     //  control the left front drive wheel
    private DcMotor rightFrontDrive;    //  control the right front drive wheel
    private DcMotor leftBackDrive;      //  control the left back drive wheel
    private DcMotor rightBackDrive;     //  control the right back drive wheel

    private DcMotor driveEncoder;       //  the Axial (front/back) Odometry Module (may overlap with motor, or may not)
    private DcMotor strafeEncoder;      //  the Lateral (left/right) Odometry Module (may overlap with motor, or may not)

    private LinearOpMode myOpMode;
    private Manipulator myArm;
    private Vision myVision;
    private IMU imu;
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    private int rawDriveOdometer    = 0; // Unmodified axial odometer count
    private int driveOdometerOffset = 0; // Used to offset axial odometer
    private int rawStrafeOdometer   = 0; // Unmodified lateral odometer count
    private int strafeOdometerOffset= 0; // Used to offset lateral odometer
    private double rawHeading       = 0; // Unmodified heading (degrees)
    private double headingOffset    = 0; // Used to offset heading

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU
    private boolean showTelemetry     = false;

    // Robot Constructor

    private static Robot instance = null;
    public boolean enabled;

    /**
     * Creating the singleton the first time, instantiating.
     */
    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(LinearOpMode opMode, Manipulator arm, Vision vision, boolean showTelemetry)
    {
        myOpMode = opMode;
        myArm = arm;
        myVision = vision;

        // Initialize the hardware variables. Note that the strings used to 'get' each
        // motor/device must match the names assigned during the robot configuration.

        // !!!  Set the drive direction to ensure positive power drives each wheel forward.
        leftFrontDrive  = setupDriveMotor("leftfront_drive", DcMotor.Direction.REVERSE);
        rightFrontDrive = setupDriveMotor("rightfront_drive", DcMotor.Direction.FORWARD);
        leftBackDrive  = setupDriveMotor( "leftback_drive", DcMotor.Direction.REVERSE);
        rightBackDrive = setupDriveMotor( "rightback_drive",DcMotor.Direction.FORWARD);
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        //  Connect to the encoder channels using the name of that channel.
        driveEncoder = myOpMode.hardwareMap.get(DcMotor.class, "axial");
        strafeEncoder = myOpMode.hardwareMap.get(DcMotor.class, "lateral");

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                             RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        setHeading(Globals.LAST_HEADING);

        // zero out all the odometry readings.
        resetOdometry();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    /**
     *   Setup a drive motor with passed parameters.  Ensure encoder is reset.
     * @param deviceName  Text name associated with motor in Robot Configuration
     * @param direction   Desired direction to make the wheel run FORWARD with positive power input
     * @return the DcMotor object
     */
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    /**
     * Read all input devices to determine the robot's motion
     * always return true so this can be used in "while" loop conditions
     * @return true
     */
    public boolean readSensors() {

        if (Globals.IS_AUTO) {
            // these are only used in Auto;
            rawDriveOdometer = driveEncoder.getCurrentPosition() * (INVERT_DRIVE_ODOMETRY ? -1 : 1);
            rawStrafeOdometer = strafeEncoder.getCurrentPosition() * (INVERT_STRAFE_ODOMETRY ? -1 : 1);
            driveDistance = (rawDriveOdometer - driveOdometerOffset) * ODOM_INCHES_PER_COUNT;
            strafeDistance = (rawStrafeOdometer - strafeOdometerOffset) * ODOM_INCHES_PER_COUNT;
        } else {
            // these are only used in Teleop;
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            turnRate    = angularVelocity.zRotationRate;
        }

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        rawHeading  = orientation.getYaw(AngleUnit.DEGREES);
        pitch       = orientation.getPitch(AngleUnit.DEGREES);
        heading     = normalizeHeading(rawHeading - headingOffset);
        Globals.LAST_HEADING = heading ;

        if (showTelemetry) {
            // myOpMode.telemetry.addData("Odom Ax:Lat", "%6d %6d", rawDriveOdometer - driveOdometerOffset, rawStrafeOdometer - strafeOdometerOffset);
            myOpMode.telemetry.addData("Dist Ax:Lat", "%5.2f %5.2f", driveDistance, strafeDistance);
            myOpMode.telemetry.addData("Head Deg:Rate", "%5.2f %5.2f", heading, turnRate);
        }
        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(distanceInches, power);   // achieve desired drive distance
        strafeController.reset(0);              // Maintain zero strafe drift
        yawController.reset();                          // Maintain last turn heading
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myArm.runArmControl();
            myOpMode.sleep(1);
        }
        stopRobot();
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param distanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void strafe(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(0.0);             //  Maintain zero drive drift
        strafeController.reset(distanceInches, power);  // Achieve desired Strafe distance
        yawController.reset();                          // Maintain last turn angle
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myArm.runArmControl();
            myOpMode.sleep(1);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {

        power = Math.abs(power);
        yawController.reset(headingDeg, power);
        while (myOpMode.opModeIsActive() && readSensors()) {

            // implement desired axis powers
            moveRobot(0, 0, yawController.getOutput(heading));

            // Time to exit?
            if (yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myArm.runArmControl();
            myOpMode.sleep(1);
        }
        stopRobot();
    }

    public void driveToTag(int desiredTagID) {

        v_driveController.reset(V_DESIRED_DISTANCE);      // achieve desired drive distance
        v_strafeController.reset(0);              // Maintain zero strafe drift
        yawController.reset();
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            boolean targetFound = false;
            AprilTagDetection desiredTag = null;

            myArm.runArmControl();  // keep the arm doing what it's meant to do;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = myVision.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if ((detection.metadata != null) && (detection.id == desiredTagID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    myOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    myOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    break;  // don't look any further.
                }
            }

            if (targetFound) {
                moveRobot(-v_driveController.getOutput((desiredTag.ftcPose.y)), v_strafeController.getOutput(desiredTag.ftcPose.x), yawController.getOutput(heading));
            } else {
                myOpMode.telemetry.addLine("Desired AprilTag NOT found");
                stopRobot();
            }

            // Time to exit?
            if (v_driveController.inPosition()) {
                break;   // Exit loop if we are in position
            }

            myOpMode.sleep(1);
        }
        stopRobot();
    }


    //  ########################  Low level control functions.  ###############################

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * Assumes all drive motors are configured forward motion when given positive power.
     * @param drive     Fwd/Rev axis power (+ve is forward)
     * @param strafe    Left/Right axis power (+ve is left)
     * @param yaw       Yaw axis power (+ve is CCW)
     */
    public void moveRobot(double drive, double strafe, double yaw){

        double lF = drive - strafe - yaw;
        double rF = drive + strafe + yaw;
        double lB = drive + strafe - yaw;
        double rB = drive - strafe + yaw;

        //normalize the motor values
        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

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
            myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            //myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
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
     * Set odometry counts and distances to zero.
     * Use this call to prepare for new motions
     */
    public void resetOdometry() {
        readSensors();
        driveOdometerOffset = rawDriveOdometer;
        driveController.reset(0);
        driveDistance = 0.0;

        strafeOdometerOffset = rawStrafeOdometer;
        strafeController.reset(0);
        strafeDistance = 0.0;
    }

    /**
     * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
     * Use this call to set a new Absolute Zero Position.  Do this at the start of Auto, or during
     * Teleop to correct any gyro drift that may occur.
     */
    public void resetHeading() {
        setHeading(0);
    }

    public void setHeading(double newHeading) {
        readSensors();
        headingOffset = rawHeading - newHeading;
        yawController.reset(newHeading);
        heading = newHeading;
        Globals.LAST_HEADING = heading ;
    }

    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    public double normalizeHeading(double aHeading) {
        while (aHeading > 180)  aHeading -= 360;
        while (aHeading <= -180) aHeading += 360;
        return aHeading;
    }

    /**
     * Set the drive telemetry on or off
     */
    public void showTelemetry(boolean show){
        showTelemetry = show;
    }
}

