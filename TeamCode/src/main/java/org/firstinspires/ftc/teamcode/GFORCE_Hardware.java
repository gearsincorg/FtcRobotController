/* Copyright (c) 2019 G-FORCE.
 * This class is used to define all the specific Hardware elements for the G-FORCE robot
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

public class GFORCE_Hardware {
    public static enum AllianceColor {
        UNKNOWN_COLOR,
        RED,
        BLUE
    }

    public static final String TAG = "Hardware";
    public static final boolean LOGGING = false;  // Set to true to add data to logfile


    /* Public OpMode members. */
    public AllianceColor allianceColor = AllianceColor.UNKNOWN_COLOR;
    public DcMotorEx leftDrive = null;
    public DcMotorEx rightDrive = null;

    public DcMotorEx frontCollector = null;
    public DcMotorEx midCollector = null;

    public DcMotorEx leftShooter = null;
    public DcMotorEx rightShooter= null;

    public DcMotorEx ringLift = null;
    public DcMotorEx ringFeed = null;

    List<LynxModule> allHubs = null;

    public Servo ringGrab = null;
    public Servo ringFinger = null;
    public Servo ringStop = null;

    public RevTouchSensor midCollectorDown = null;

    OpenCvCamera webcam;

    //public DigitalChannel leftLimit = null;
    //public DigitalChannel rightLimit = null;


    public static BNO055IMU imu = null;

    public final double MAX_VELOCITY        = 2500;  // Counts per second
    public final double MAX_VELOCITY_MMPS   = 2540;  // MM Per Second
    public final double AUTO_ROTATION_DPS   = 2540;   // Degrees per second

    public final double ACCELERATION_LIMIT  = 1000;  // MM per second per second  was 1524

    public final double YAW_GAIN            = 0.010;  // Rate at which we respond to heading error 0.013
    public final double LATERAL_GAIN        = 0.0025; // Distance from x axis that we start to slow down. 0027
    public final double AXIAL_GAIN          = 0.0015; // Distance from target that we start to slow down. 0017

    // SERVO CONSTANTS


    // Driving constants Yaw heading
    final double HEADING_GAIN       = 0.015;  // Was 0.012
    final double TURN_RATE_TC       = 0.6;
    final double STOP_TURNRATE      = 0.020;
    final double GYRO_360_READING   = 360.0;
    final double GYRO_SCALE_FACTOR  = 360.0 / GYRO_360_READING;

    final double YAW_IS_CLOSE = 2.0;  // angle within which we are "close"

    final double AXIAL_ENCODER_COUNTS_PER_MM   = 0.958; // was 0.8602
    final double LATERAL_ENCODER_COUNTS_PER_MM = 0.958;  // was 0.9134



    // Robot states that we share with others
    public double axialMotion = 0;
    public double lateralMotion = 0;
    public double currentHeading = 0;


    private static LinearOpMode myOpMode = null;

    // Sensor Read Info.
    private int   encoderLB;
    private int   encoderLF;
    private int   encoderRB;
    private int   encoderRF;

    private int startLeftBack = 0;
    private int startLeftFront = 0;
    private int startRightBack = 0;
    private int startRightFront = 0;
    private int deltaLeftBack = 0;
    private int deltaLeftFront = 0;
    private int deltaRightBack = 0;
    private int deltaRightFront = 0;

    private double driveAxial = 0;
    private double driveYaw = 0;
    private double driveLateral = 0;
    private double startTime = 0;

    // gyro
    private static double lastHeading = 0;
    private double lastCycle = 0;
    private double intervalCycle = 0;
    private double filteredTurnRate = 0;
    private double integratedZAxis = 0;
    private double adjustedIntegratedZAxis = 0;
    private double headingSetpoint = 0;

    // Scoring Status variables



    private int timeoutSoundID = 0;

    /* local OpMode members. */
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime cycleTime = new ElapsedTime();
    private ElapsedTime navTime = new ElapsedTime();
    private ElapsedTime craneTime = new ElapsedTime();
    private ElapsedTime liftTime = new ElapsedTime();

    /* Constructor */
    public GFORCE_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode) {
        // Save reference to Hardware map
        myOpMode = opMode;

        // Define and Initialize Motors
        leftDrive = configureMotor("left_drive", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive = configureMotor("right_drive", DcMotor.Direction.FORWARD,DcMotor.RunMode.RUN_USING_ENCODER);
        frontCollector = configureMotor("front_collector",DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);          //Was m1 in CollectorTest
        midCollector = configureMotor("mid_collector",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);              //Was m2 in CollectorTest
        //leftShooter = configureMotor ("left_shooter",DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightShooter = configureMotor ("right_shooter",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //ringLift = configureMotor ("ring_lift",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        //ringFeed = configureMotor ("ring_feed",DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Define and Initialize Sensors
        midCollectorDown = myOpMode.hardwareMap.get(RevTouchSensor.class,"midTouch");

        //Define and Initialize Ring Servos
        ringGrab = myOpMode.hardwareMap.get(Servo.class,"ring_grab");
        ringFinger = myOpMode.hardwareMap.get(Servo.class,"ring_finger");
        ringStop = myOpMode.hardwareMap.get(Servo.class,"ring_stop");


        // Set all Expansion hubs to use the MANUAL Bulk Caching mode
        allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        resetEncoders();


        timeoutSoundID = myOpMode.hardwareMap.appContext.getResources().getIdentifier("ss_siren", "raw", myOpMode.hardwareMap.appContext.getPackageName());
        if (timeoutSoundID != 0) {
            SoundPlayer.getInstance().preload(myOpMode.hardwareMap.appContext, timeoutSoundID);
        }

        //Do a full gyro init if it has never ben initialized
        //Else just reconnect to it
        if (imu == null) {
            lastHeading = 0;
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        setHeading(lastHeading);

        // Set all motors to zero power
        stopRobot();

    }

    // Configure a motor
    public DcMotorEx configureMotor( String name, DcMotor.Direction direction, DcMotor.RunMode mode) {
        DcMotorEx motorObj = myOpMode.hardwareMap.get(DcMotorEx.class, name);
        motorObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorObj.setDirection(direction);
        motorObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorObj.setMode(mode);
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
        double endingTime = runTime.seconds() + timeOutSec;
        double absMm = Math.abs(mm);

        // Reverse axial directions for blue autonomous if flipOnBlue is true
        if ((allianceColor == AllianceColor.BLUE) && flipOnBlue) {
            vel = -vel;
        }

        // If we are moving backwards, set vel negative
        if ((mm * vel) < 0.0) {
            vel = -Math.abs(vel);
        } else {
            vel = Math.abs(vel);
        }

        RobotLog.ii(TAG, String.format("DAV D:V:H %5.0f:%5.0f ", mm, vel, heading));

        //Save the current position
        startMotion();

        // Loop until the robot has driven to where it needs to go
        // Remember to call updateMotion() once per loop cycle.
        while (myOpMode.opModeIsActive() && updateMotion() &&
                (Math.abs(axialMotion) < absMm) &&
                (runTime.seconds() < endingTime)) {
            if (LOGGING) RobotLog.ii(TAG, String.format("DAV A:L %5.0f:%5.0f ", axialMotion, lateralMotion));
            setAxialVelocity(getProfileVelocity(vel, getAxialMotion(), absMm));
            setLateralVelocity(-lateralMotion);  // Reverse any drift
            setYawVelocityToHoldHeading(heading);
            moveRobotVelocity();
            showEncoders();
        }

        stopRobot();
        updateMotion();
        RobotLog.ii(TAG, String.format("DAV Last A:L %5.0f:%5.0f ", axialMotion, lateralMotion));

        // Return true if we have not timed out
        boolean success = (runTime.seconds() < endingTime) ;
        if (!success) {
            playTimoutSound();
        }
        return (success);
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

        double endingTime = runTime.seconds() + timeOutSec;
        double absMm = Math.abs(mm);

        RobotLog.ii(TAG, String.format("DLV D:V:H %5.0f : %5.0f : %5.0f ", mm, vel, heading));

        // Reverse Lateral directions for blue autonomous and when flipOnBlue is true
        if ((allianceColor == AllianceColor.BLUE) && flipOnBlue) {
          vel = -vel;
        }

        // If we are moving backwards, set vel negative
         if ((mm * vel) < 0.0) {
            vel = -Math.abs(vel);
        } else {
            vel = Math.abs(vel);
        }

        //Save the current position
        startMotion();

        // Loop until the robot has driven to where it needs to go
        // Remember to call updateMotion() once per loop cycle.
        while (myOpMode.opModeIsActive() && updateMotion() &&
                (Math.abs(lateralMotion) < absMm) &&
                (runTime.seconds() < endingTime)) {
            if (LOGGING) RobotLog.ii(TAG, String.format("DLV A:L %5.0f:%5.0f ", axialMotion, lateralMotion));
            setAxialVelocity(-axialMotion);  //  Reverse any drift
            setLateralVelocity(getProfileVelocity(vel, getLateralMotion(), absMm));
            setYawVelocityToHoldHeading(heading);
            moveRobotVelocity();
            showEncoders();
        }

        stopRobot();
        RobotLog.ii(TAG, String.format("DLV Last A:L %5.0f:%5.0f ", axialMotion, lateralMotion));

        // Return true if we have not timed out
        boolean success = (runTime.seconds() < endingTime) ;
        if (!success) {
            playTimoutSound();
        }
        return (success);
    }

    //
    public double getProfileVelocity(double topVel, double dTraveled, double dGoal) {
        double profileVelocity = 0;

        // Make all distances positive
        double absdTraveled = Math.abs(dTraveled);
        double absdGoal     = Math.abs(dGoal);
        double absTopVel = Math.abs(topVel);
        double currentVel = leftDrive.getVelocity();

        // Treat the profile as an acceleration half and a deceleration half, based on distance traveled.
        // Determine the velocity, then just clip the requested velocity based on the requested top speed.
        // While Accelerating, V = Alimit * Time * 2
        // While Decelerating, V = ALimit * SQRT(2 * dRemaining / ALimit )

        if (absdTraveled < (absdGoal / 2.0)) {
            // We are accelerating, give a little boost
            profileVelocity = (ACCELERATION_LIMIT * getMotionTime() * 2) ;

        } else if (absdTraveled < absdGoal ) {
            // We are Decelerating
            double dRemaining = Range.clip((absdGoal - absdTraveled), 0, absdGoal); // Don't let this go negative.
            profileVelocity = ACCELERATION_LIMIT * Math.sqrt(2 * dRemaining / ACCELERATION_LIMIT);
        } else {
            // We Are Stopped
            profileVelocity = 0.0;
        }

            //Brake if velocity is too high
        if (Math.abs(currentVel) > profileVelocity) {
            profileVelocity = 0;
        }

        // Make sure the final velocity sign is correct.
        profileVelocity = Range.clip(profileVelocity, 0, absTopVel) * Math.signum(topVel);

        if (LOGGING) Log.d(TAG, String.format("GVP T:V:D:A %5.3f %4.2f %5.2f %5.2f",
                 getMotionTime(), profileVelocity, absdTraveled, currentVel / AXIAL_ENCODER_COUNTS_PER_MM));

        return (profileVelocity);
    }

    // Common location to read all sensors
    public void readSensors() {

        // If we are exiting, don't read bulk data
        if (!myOpMode.isStopRequested()) {
            // Clear the BulkCache once per control cycle
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            encoderLF =         leftDrive.getCurrentPosition();
            encoderRF =         rightDrive.getCurrentPosition();

            getHeading();

            intervalCycle = cycleTime.milliseconds() - lastCycle;
            lastCycle = cycleTime.milliseconds();
        }
    }

    //Get the current encoder counts of the drive motors
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

    public double getAxialMotion() {
        // NOTE:  Must call updateMotion() once before calling this method;
        return (axialMotion);
    }

    public double getLateralMotion() {
        // NOTE:  Must call updateMotion() once before calling this method;
        return (lateralMotion);
    }

    // Turn with both wheels
    public boolean turnToHeading(double heading, double timeOutSEC) {
        return generalRotationControl(heading, timeOutSEC, false);
    }

    public boolean sleepAndHoldHeading(double heading, double timeOutSEC) {
        return generalRotationControl(heading, timeOutSEC, true);
    }


    public boolean generalRotationControl(double heading, double timeOutSEC, boolean waitFullTimeout) {
        boolean inPosition = false;  // needed to run at least one control cycle.
        boolean timedOut = false;

        setHeadingSetpoint(heading);
        setAxialVelocity(0);
        setLateralVelocity(0);
        navTime.reset();

        while (myOpMode.opModeIsActive() &&
                updateMotion() &&
                (navTime.time() < timeOutSEC) &&
                (!inPosition || waitFullTimeout)) {

            inPosition = setYawVelocityToHoldHeading(heading);

            //Brake if velocity is too high
            if (Math.abs(rightDrive.getVelocity()) > Math.abs(driveYaw)) {
                driveYaw = 0;
            }

            moveRobotVelocity();
            showEncoders();
        }
        stopRobot();

        if (!waitFullTimeout && (navTime.time() > timeOutSEC)) {
            timedOut = true;
            playTimoutSound();
        }

        return (!timedOut);
    }

    /*
    //Turning code from Rover Ruckus when we had two omni wheels and two traction wheels
    //Need to change to Velocity
    // turn with both wheels
    public boolean turnToHeading(double newSetpoint, double timeOutSEC) {
        // Flip translation and rotations if we are RED
        setHeadingSetpoint(newSetpoint);
        navTime.reset();
        while (myOpMode.opModeIsActive() &&
                (navTime.time() < timeOutSEC)&&
                !setYawVelocityToHoldHeading() &&
                (runningAuto || runningAuto())) {
            moveRobotVelocity();
        }
        if (navTime.time() > timeOutSEC) {
            playTimoutSound();
        }
        stopRobot();
        return (navTime.time() < timeOutSEC);
    }

    // sweeping turn with one wheel or the other
    public boolean sweepToHeading(double newSetpoint, double timeOutSEC, boolean moveLeftWheel) {
        // Flip translation and rotations if we are RED
        setHeadingSetpoint(newSetpoint);
        navTime.reset();
        while (myOpMode.opModeIsActive() &&
                (navTime.time() < timeOutSEC)&&
                !setYawVelocityToHoldHeading() &&
                (runningAuto || runningAuto())) {
            if (moveLeftWheel) {
                // Set drive motor power levels.
                leftDrive.setPower(Range.clip(-driveYaw * 2, -1, 1));
                rightDrive.setVelocity(0);
            } else {
                leftDrive.setVelocity(0);
                rightDrive.setPower(Range.clip(driveYaw * 2, -1, 1));
            }
        }
        if (navTime.time() > timeOutSEC) {
            playTimoutSound();
        }
        stopRobot();
        return (navTime.time() < timeOutSEC);
    }
    */









    public void playTimoutSound() {
        if (timeoutSoundID != 0) {
            SoundPlayer.getInstance().startPlaying(myOpMode.hardwareMap.appContext, timeoutSoundID);
        }
    }

    public void resetEncoders() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void showEncoders() {
        myOpMode.telemetry.addData("Heading", "%+3.1f (%.0fmS)", adjustedIntegratedZAxis, intervalCycle);
        myOpMode.telemetry.addData("axes",  "A:L:Y %6.0f %6.0f %6.0f", driveAxial, driveLateral,driveYaw);
        myOpMode.telemetry.addData("motion","axial %6.1f, lateral %6.1f", getAxialMotion(), getLateralMotion());
        myOpMode.telemetry.update();
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
    }

    // --------------------------------------------------------------------
    // Heading/Gyro Control
    // --------------------------------------------------------------------

    public double getHeading() {

        Orientation angles;
        double heading;
        double deltaH;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;

        if (heading != lastHeading) {
            // determine change in heading and apply to integrated Z
            deltaH = heading - lastHeading;
            if (deltaH < -180)
                deltaH += 360;
            else if (deltaH > 180)
                deltaH -= 360;

            integratedZAxis += deltaH;
            lastHeading = heading;
        }
        adjustedIntegratedZAxis = integratedZAxis * GYRO_SCALE_FACTOR;
        currentHeading = adjustedIntegratedZAxis;
        return (currentHeading);
    }

    public void setHeadingSetpoint(double newSetpoint) {
        headingSetpoint = newSetpoint;
    }

    public boolean setYawVelocityToHoldHeadingWithUpdate(double newSetpoint) {
        getHeading();
        setHeadingSetpoint(newSetpoint);
        return (setYawVelocityToHoldHeading());
    }

    public boolean setYawVelocityToHoldHeadingWithUpdate() {
        getHeading();
        return setYawVelocityToHoldHeading();
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

        if (LOGGING) RobotLog.ii(TAG, String.format("sYVTHH SP:CH:Y %6.3f %6.3f %6.3f" , headingSetpoint, currentHeading, yaw));

        setYawVelocity(yaw * AUTO_ROTATION_DPS);
        return (Math.abs(error) < YAW_IS_CLOSE);
    }

    public void setHeading(double newHeading) {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // set new integrated value and save current heading for future calculations
        lastHeading = angles.firstAngle;
        integratedZAxis = newHeading;
        headingSetpoint = newHeading;
    }

    public void resetHeading() {
        setHeading(0.0);
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

    public boolean notTurning() {

        AngularVelocity velocities;
        velocities = imu.getAngularVelocity();
        double rate = velocities.zRotationRate;

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

        leftDrive.setVelocity(leftFrontVel);
        rightDrive.setVelocity(rightFrontVel);

        // Log.d("G-FORCE AUTO", String.format("M %5.1f %5.1f %5.1f %5.1f ", leftFrontVel, rightFrontVel, leftBackVel, rightBackVel));
    }

    public void setAxialPower (double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void stopRobot() {
        moveRobotVelocity(0, 0, 0);
    }

    public void runCollectors(double leftPower, double rightPower) {
        frontCollector.setPower(leftPower);
        midCollector.setPower(rightPower);
    }

    public void runCollector(double Power) {
        frontCollector.setPower(Power);
        midCollector.setPower(Power);
    }

    // ========================================================
    // ----               SERVO Methods
    // ========================================================









/*
    // --------------------------------------------------------------
    // Lift Control
    // --------------------------------------------------------------
    public void setLiftSetpoint(double angle)  {
        liftSetpoint = angle;
    }

    public void lockLiftInPlace(){
        liftSetpoint = liftAngle;
        liftInPosition = true;
    }

    public double getLiftAngle() {
        return liftAngle;
    }

    public void zeroLiftEncoders () {
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runLiftControl() {

        if (LOGGING) RobotLog.ii(TAG, String.format("RLC state:time:setpoint:angle %s, %6.3f %6.3f %6.3f",
                                    liftState , liftTime.time(), liftSetpoint, liftAngle));

        switch (liftState) {
            case AUTO:

                if (myOpMode.gamepad2.back && myOpMode.gamepad2.start) {
                    liftSetpoint += LIFT_HOME;
                    liftState = LiftControl.HOME_RAISING;
                    liftTime.reset();  //
                }
                else if (myOpMode.gamepad2.dpad_up && (liftAngle < LIFT_UPPER_LIMIT)) {
                    liftSetpoint = liftAngle + 2 + (2 * liftTime.time());  // speed up lift over time

                }
                else if (myOpMode.gamepad2.dpad_down && (liftAngle > LIFT_LOWER_LIMIT)) {
                    liftSetpoint = liftAngle - 2 - (2 * liftTime.time());  // speed up lift over time;
                }
                else {
                    liftTime.reset();

                    // Run lift using left Joystick
                    if(Math.abs(myOpMode.gamepad2.left_stick_y) > 0.1 ) {
                        liftSetpoint -= (myOpMode.gamepad2.left_stick_y * 1.5);
                    }
                }

                setLiftPower();
                break;

            case HOME_RAISING:
                if ((liftInPosition && (liftTime.time() > 0.8)) || (liftTime.time() > 1.0)) {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftTime.reset();
                    liftState = LiftControl.HOME_LOWERING;
                } else {
                    setLiftPower();
                }
                break;

            case HOME_LOWERING:
                leftLift.setPower(leftLimitTripped ? 0 : -0.05);
                rightLift.setPower(rightLimitTripped ? 0 : -0.05);

                if ((leftLimitTripped && rightLimitTripped) || (liftTime.time() > 2.0)) {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    zeroLiftEncoders (); // also setes mode
                    liftSetpoint = LIFT_LOWER_LIMIT;
                    liftState = LiftControl.AUTO;
                }
        }
    }

    public void releaseCollectorArms() {
        beginReleaseCollectorArms();

        while (liftState != LiftControl.AUTO) {
            readSensors();
            runLiftControl();
        }
    }

    public void beginReleaseCollectorArms() {
        readSensors();
        liftSetpoint += COLLECTOR_RELEASE;
        liftState = LiftControl.HOME_RAISING;
        liftInPosition = false;
        liftTime.reset();  //
        setLiftPower();
    }

    private void setLiftPower () {
        double leftPower = 0;
        double rightPower = 0;

        // Restrict Lift Setpoint;
        liftSetpoint = Range.clip(liftSetpoint, LIFT_LOWER_LIMIT, LIFT_UPPER_LIMIT);

        // Determine lift position error
        double leftLiftError = liftSetpoint - leftLiftAngle;
        double rightLiftError = liftSetpoint - rightLiftAngle;

        leftPower = Range.clip(leftLiftError * LIFT_GAIN, AUTO_LOWER_POWER, AUTO_RAISE_POWER);
        rightPower = Range.clip(rightLiftError * LIFT_GAIN, AUTO_LOWER_POWER, AUTO_RAISE_POWER);

        if ((leftPower < 0) && leftLimitTripped) {
            leftPower = 0;
        }

        if ((rightPower < 0) && rightLimitTripped) {
            rightPower = 0;
        }

        liftInPosition = (Math.abs(leftLiftError + rightLiftError) < LIFT_IN_LIMIT);
        leftLift.setPower(leftPower);
        rightLift.setPower(rightPower);
    }

    public boolean extendJustClicked() {
        boolean clicked = false;

        clicked = (myOpMode.gamepad2.a && !lastExtend);
        lastExtend = myOpMode.gamepad2.a;
        return (clicked);
    }

 */
}
