/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class DriveSubsystem {
    // Adjust these numbers to suit your robot.
    private final double  COUNTS_PER_REV            = 384.5;   //  GoBilda 435 RPM motor (13.7:1 gear)
    private final double  WHEEL_DIAMETER_IN         = 3.75;    //  GoBilda 96mm Mecanum Wheel

    private final double  AXIAL_INCHES_PER_COUNT    = (Math.PI * WHEEL_DIAMETER_IN) / COUNTS_PER_REV;
    private final double  LATERAL_INCHES_PER_COUNT  = AXIAL_INCHES_PER_COUNT * 0.866;

    private final int FRONT_SONAR = 4;


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

    private static final double YAW_GAIN            = 0.018;    // Strength of Yaw position control
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    // Public Members
    public double driveDistance     = 0; // scaled axial distance (+ = forward)
    public double strafeDistance    = 0; // scaled lateral distance (+ = left)
    public double heading           = 0; // Latest Robot heading from IMU
    public double frontRange = 0;
    public double backRange = 0;

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public ProportionalControl driveController     = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public ProportionalControl strafeController    = new ProportionalControl(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    // ---  Private Members

    // Hardware interface Objects
    private DcMotor leftFrontDrive;     //  control the left front drive wheel
    private DcMotor rightFrontDrive;    //  control the right front drive wheel
    private DcMotor leftBackDrive;      //  control the left back drive wheel
    private DcMotor rightBackDrive;     //  control the right back drive wheel

    private int encoderLF;              // Encoder value for front left wheel
    private int encoderRF;              // Encoder value for front right wheel
    private int encoderLB;              // Encoder value for back left wheel
    private int encoderRB;              // Encoder value for back right wheel

    private OctoQuad octoquad;

    private LinearOpMode myOpMode;
    private IMU imu;
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    private int startLeftFront = 0;
    private int startRightFront = 0;
    private int startLeftBack = 0;
    private int startRightBack = 0;
    private int deltaLeftFront = 0;
    private int deltaRightFront = 0;
    private int deltaLeftBack = 0;
    private int deltaRightBack = 0;

    private double rawHeading       = 0; // Unmodified heading (degrees)
    private double headingOffset    = 0; // Used to offset heading

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU (deg / sec)
    private boolean showTelemetry     = false;

    // Robot Constructor
    public DriveSubsystem(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry)
    {
        // Initialize the hardware variables. Note that the strings used to 'get' each
        // motor/device must match the names assigned during the robot configuration.

        // !!!  Set the drive direction to ensure positive power drives each wheel forward.
        leftFrontDrive  = setupDriveMotor("leftfront_drive", DcMotor.Direction.REVERSE);
        rightFrontDrive = setupDriveMotor("rightfront_drive", DcMotor.Direction.FORWARD);
        leftBackDrive  = setupDriveMotor( "leftback_drive", DcMotor.Direction.REVERSE);
        rightBackDrive = setupDriveMotor( "rightback_drive",DcMotor.Direction.FORWARD);
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        // Connect to the OctoQuad by looking up its name in the hardwareMap.
        octoquad = myOpMode.hardwareMap.get(OctoQuad.class, "octoquad");

        // Clear out all prior settings and encoder data before setting up desired configuration
        // Assume first 4 channels are relative encoders and the next 4 are absolute encoders
        octoquad.resetEverything();
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);


        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                             RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // zero out all the odometry readings.
        startMotion();

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
        // Read motor encoders for each wheel
        encoderLF =         leftFrontDrive.getCurrentPosition();
        encoderRF =         rightFrontDrive.getCurrentPosition();
        encoderLB =         leftBackDrive.getCurrentPosition();
        encoderRB =         rightBackDrive.getCurrentPosition();

        updateMotion();  // determine how robot has moved from most recent startMotion() call;

        // read the IMU data.
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        rawHeading  = orientation.getYaw(AngleUnit.DEGREES);
        heading     = rawHeading - headingOffset;
        turnRate    = angularVelocity.zRotationRate;

        OctoQuad.EncoderDataBlock encoderDataBlock = new OctoQuad.EncoderDataBlock();
        octoquad.readAllEncoderData(encoderDataBlock);
        frontRange = encoderDataBlock.positions[FRONT_SONAR] / 25.4;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Dist Ax:Lat", "%5.2f %5.2f", driveDistance, strafeDistance);
            myOpMode.telemetry.addData("Head Deg:Rate", "%5.2f %5.2f", heading, turnRate);
            myOpMode.telemetry.addData("Sonar Distance", "%.1f inches", frontRange);
        }
        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    // Initialize all the encoder starting values for the next motion
    public void startMotion() {
        readSensors();  // get the latest data
        startLeftBack = encoderLB;  // Save the current values as the start values.
        startLeftFront = encoderLF;
        startRightBack = encoderRB;
        startRightFront = encoderRF;
        updateMotion();  // Update the derived motion data.
    }

    public void updateMotion() {
        deltaLeftFront = encoderLF - startLeftFront;
        deltaRightFront = encoderRF - startRightFront;
        deltaLeftBack = encoderLB - startLeftBack;
        deltaRightBack = encoderRB - startRightBack;

        driveDistance  = ((deltaLeftFront + deltaRightFront + deltaLeftBack + deltaRightBack ) / 4) * AXIAL_INCHES_PER_COUNT;
        strafeDistance = ((-deltaLeftFront + deltaRightFront + deltaLeftBack - deltaRightBack) / 4) * LATERAL_INCHES_PER_COUNT;
    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        startMotion();

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
            myOpMode.sleep(10);
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
        startMotion();

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
            myOpMode.sleep(10);
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
            myOpMode.sleep(10);
        }
        stopRobot();
    }


    //  ########################  Low level control functions.  ###############################

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * @param drive     Fwd/Rev axis power     	+ve means forward
     * @param strafe    Left/Right axis power	+ve means left
     * @param yaw       Yaw axis power			+ve means CCW
     */
    public void moveRobot(double drive, double strafe, double yaw){

        double lF = drive - strafe - yaw;
        double rF = drive + strafe + yaw;
        double lB = drive + strafe - yaw;
        double rB = drive - strafe + yaw;

        //normalize the motor values.  Make sure no motor values exceeds 100% power
        double max = 1.0;
        max = Math.max(max, Math.abs(lF));
        max = Math.max(max, Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

		// Scale by the max value (defaults to 1.0)
        lF /= max;
        rF /= max;
        lB /= max;
        rB /= max;

        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

        if (showTelemetry) {
            myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
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
     * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
     */
    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        yawController.reset(0);
        heading = 0;
    }

    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    /**
     * Set the drive telemetry on or off
     */
    public void showTelemetry(boolean show){
        showTelemetry = show;
    }
}

//****************************************************************************************************
//****************************************************************************************************

