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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class GFORCE_Hardware {
    public static enum AllianceColor {
        UNKNOWN_COLOR,
        RED,
        BLUE
    }

    public static final String TAG = "Hardware";
    public static final boolean LOGGING = true;  // Set to true to add data to logfile

    /* Public OpMode members. */
    public AllianceColor allianceColor = AllianceColor.UNKNOWN_COLOR;
    public DcMotorEx leftDrive = null;
    public DcMotorEx rightDrive = null;

    public DcMotorEx frontCollector = null;
    public DcMotorEx midCollector = null;

    public DcMotorEx leftShooter = null;
    public DcMotorEx rightShooter = null;

    public DcMotorEx ringLift = null;
    public DcMotorEx ringFeed = null;

    List<LynxModule> allHubs = null;

    public Servo leftWobbleGrab = null;
    public Servo rightWobbleGrab = null;
    public Servo ringStop = null;

    public RevTouchSensor midCollectorDown = null;

    OpenCvCamera webcam;

    public DigitalChannel leftLimit = null;
    public DigitalChannel rightLimit = null;

    public static BNO055IMU imu = null;

    public final double MAX_VELOCITY = 2700;  // Counts per second
    public final double AUTO_ROTATION_DPS = 2540;  // Degrees per second

    public final double MAX_AXIAL_MMPS = 2540;  // MM Per Second
    public final double MAX_YAW_MMPS = 1000;  // MM Per Second
    public final double ACCELERATION_LIMIT = 1500;  // MM per second per second

    public final double INITIAL_SHOOTER_SPEED = 2400; // CPS
    public final double SHOOTER_SPEED_TEST    =  100; // CPS


    public final double AXIAL_GAIN = 0.0015; // Distance from target that we start to slow down. 0017
    public final double YAW_GAIN = 0.010;  // Rate at which we respond to heading error 0.013

    // SERVO CONSTANTS

    // Driving constants Yaw heading
    final double HEADING_GAIN = 0.010;  // Was 0.012
    final double TURN_RATE_TC = 0.6;
    final double STOP_TURNRATE = 0.020;
    final double GYRO_360_READING = 360.0;
    final double GYRO_SCALE_FACTOR = 360.0 / GYRO_360_READING;

    final double YAW_IS_CLOSE = 2.0;  // angle within which we are "close"

    final double AXIAL_ENCODER_COUNTS_PER_MM = 1.78; // 537.6 Counts per (96 * 3.1415) mm

    // Robot states that we share with others
    public double axialMotion = 0;
    public double currentHeading = 0;


    private static LinearOpMode myOpMode = null;

    // Sensor Read Info.
    private int encoderLeft;
    private int encoderRight;

    private int startLeft = 0;
    private int startRight = 0;
    private int deltaLeft = 0;
    private int deltaRight = 0;

    private double driveAxial = 0;
    private double driveYaw = 0;
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
    private double targetSpinnerSpeed = 0;

    /* local OpMode members. */
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime cycleTime = new ElapsedTime();
    private ElapsedTime navTime = new ElapsedTime();

    //VuForia Key
    public VuforiaLocalizer vuforia;
    public static final String VUFORIA_KEY =
            "ASFl1ib/////AAABmdtl1FqwZUIEqtOW/F+xX70YsCPMRYbusW+Av5TpUTDuB3VJT4z6ju8tkAzSKLD0cIwdp/o/3ggJzx27+OsIHWn8OTNfsAtxIzQVSCa75gI76/v006khzWpGV1wmdoEgK7JkvEns6BCzmgfSBSThg70Ej42wDF7l5FuIXUhm/AAMJ7sHLlMl5BboZg/vRyNRFTbEbFLyj98DOwLlaNl9DvUtf5bGBOHwFCNOBX8vlxWVU3aZZpGNxNTX/KyZ84TWECIxg8SeRSz3QcBEwsBYX97HXfj4nJxn93u8m5SZmoHF11MPkV0tlqemRwrCy/MJ3eGB3WCJ+MEeCAYeVa30E+WEkVTiFQAo4WW3vKuEVuBc";
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables targetsUltimateGoal;
    List<VuforiaTrackable> allTrackables;

    private float phoneYRotate = -90;
    private float phoneXRotate = 0;
    private float phoneZRotate = 0;
    final float CAMERA_FORWARD_DISPLACEMENT = 160.0f;   // eg: Camera is 160 mm in front of robot-center
    final float CAMERA_VERTICAL_DISPLACEMENT = 110.0f;   // eg: Camera is 8 mm above ground
    final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
    private static final float mmTargetHeight = 150.0f;          // the height in mm of the center of the target image above the floor


    // Class Members
    public OpenGLMatrix lastLocation = null;
    public boolean targetVisible = false;

    //Wobble Goal Servo Positions
    public final double LEFT_GOAL_GRAB = 0.48;
    public final double RIGHT_GOAL_GRAB = 0.52;

    public final double LEFT_GOAL_RELEASE = 0.80;
    public final double RIGHT_GOAL_RELEASE = 0.15;

    public final double RING_STOP    = 0.0;
    public final double RING_RELEASE = 1.0;

    /* Constructor */
    public GFORCE_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode) {
        // Save reference to Hardware map
        myOpMode = opMode;

        // Define and Initialize Motors
        leftDrive = configureMotor("left_drive", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive = configureMotor("right_drive", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        frontCollector = configureMotor("front_collector", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        midCollector = configureMotor("mid_collector", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter = configureMotor("left_shooter", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter = configureMotor("right_shooter", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
        ringLift = configureMotor("ring_lift", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        ringFeed = configureMotor("ring_feed", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Define and Initialize Sensors
        midCollectorDown = myOpMode.hardwareMap.get(RevTouchSensor.class, "midTouch");

        //Define and Initialize Ring Servos
        leftWobbleGrab = myOpMode.hardwareMap.get(Servo.class, "left_wobble");
        rightWobbleGrab = myOpMode.hardwareMap.get(Servo.class, "right_wobble");
        ringStop = myOpMode.hardwareMap.get(Servo.class, "ring_stop");
        releaseWobbleGoal();
        stopRings();

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

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void activateVuforiaTargets() {
        //  Instantiate the Vuforia engine
        if (vuforia == null) {
            initVuforia();
        }
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        for (VuforiaTrackable trackable : allTrackables) {
            trackable.setLocation(OpenGLMatrix
                    .translation(0, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
            /**  Let all the trackable listeners know where the phone is.  */
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsUltimateGoal.activate();
    }

    public void deactivateVuforiaTargets() {
        targetsUltimateGoal.deactivate();
    }


    // Configure a motor
    public DcMotorEx configureMotor(String name, DcMotor.Direction direction, DcMotor.RunMode mode) {
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
    public boolean driveAxialVelocity(double mm, double heading, double vel, double timeOutSec) {
        double endingTime = runTime.seconds() + timeOutSec;
        double absMm = Math.abs(mm);

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
            if (LOGGING) RobotLog.ii(TAG, String.format("DAV Ax %5.0f ", axialMotion));
            setAxialVelocity(getProfileVelocity(vel, getAxialMotion(), absMm));
            setYawVelocityToHoldHeading(heading);
            moveRobotVelocity();
            showEncoders();
        }

        stopRobot();
        updateMotion();
        RobotLog.ii(TAG, String.format("DAV Last Ax: %5.0f ", axialMotion));

        // Return true if we have not timed out
        boolean success = (runTime.seconds() < endingTime);
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
        double absdGoal = Math.abs(dGoal);
        double absTopVel = Math.abs(topVel);
        double currentVel = leftDrive.getVelocity() / AXIAL_ENCODER_COUNTS_PER_MM;

        // Treat the profile as an acceleration half and a deceleration half, based on distance traveled.
        // Determine the velocity, then just clip the requested velocity based on the requested top speed.
        // While Accelerating, V = Alimit * Time * 2
        // While Decelerating, V = ALimit * SQRT(2 * dRemaining / ALimit )

        if (absdTraveled < (absdGoal / 2.0)) {
            // We are accelerating, give a little boost
            profileVelocity = (ACCELERATION_LIMIT * getMotionTime() * 2);

        } else if (absdTraveled < absdGoal) {
            // We are Decelerating
            double dRemaining = Range.clip((absdGoal - absdTraveled), 0, absdGoal); // Don't let this go negative.
            profileVelocity = ACCELERATION_LIMIT * Math.sqrt(2 * dRemaining / ACCELERATION_LIMIT);
        } else {
            // We Are Stopped
            profileVelocity = 0.0;
        }

        //Brake if velocity is too high
        //if (Math.abs(currentVel) > profileVelocity) {
        //    profileVelocity = 0;
        //}

        // Make sure the final velocity sign is correct.
        profileVelocity = Range.clip(profileVelocity, 0, absTopVel) * Math.signum(topVel);

        if (LOGGING) Log.d(TAG, String.format("GVP T:V:D:A %5.3f %4.2f %5.2f %5.2f",
                getMotionTime(), profileVelocity, absdTraveled, currentVel));

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

            encoderLeft = leftDrive.getCurrentPosition();
            encoderRight = rightDrive.getCurrentPosition();

            getHeading();

            intervalCycle = cycleTime.milliseconds() - lastCycle;
            lastCycle = cycleTime.milliseconds();
        }
    }

    //Get the current encoder counts of the drive motors
    public void startMotion() {
        readSensors();
        startLeft = encoderLeft;
        startRight = encoderRight;
        deltaLeft = 0;
        deltaRight = 0;
        startTime = runTime.time();
    }

    public boolean updateMotion() {
        readSensors();
        deltaLeft = encoderLeft - startLeft;
        deltaRight = encoderRight - startRight;
        axialMotion = ((deltaLeft + deltaRight) / 2);
        axialMotion /= AXIAL_ENCODER_COUNTS_PER_MM;
        return (true);
    }

    public double getMotionTime() {
        return (runTime.time() - startTime);
    }

    public double getAxialMotion() {
        // NOTE:  Must call updateMotion() once before calling this method;
        return (axialMotion);
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
        navTime.reset();

        while (myOpMode.opModeIsActive() &&
                updateMotion() &&
                (navTime.time() < timeOutSEC) &&
                (!inPosition || waitFullTimeout)) {

            inPosition = setYawVelocityToHoldHeading(heading);

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
        // myOpMode.telemetry.addData("Req Vel (mmPS)",  "A:Y %6.0f %6.0f ", driveAxial, driveYaw);
        myOpMode.telemetry.addData("Act Vel (MMPS)", "L:R %6.0f %6.0f ", leftDrive.getVelocity() / AXIAL_ENCODER_COUNTS_PER_MM, rightDrive.getVelocity() / AXIAL_ENCODER_COUNTS_PER_MM);
        myOpMode.telemetry.addData("motion (mm)", "axial %6.1f", getAxialMotion());
        //myOpMode.telemetry.addData("Drive (counts)","Left %6d, Right %6d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        myOpMode.telemetry.addData("Shooter (CPS)", "Left %6.0fd, Right %6.0f", leftShooter.getVelocity(), rightShooter.getVelocity());
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
        double error = normalizeHeading(headingSetpoint - currentHeading);
        double yaw = Range.clip(error * HEADING_GAIN, -0.25, 0.25);

        //myOpMode.telemetry.addData("holdHeading","sYVTHH SP:CH:Y %6.3f %6.3f %6.3f" , headingSetpoint, currentHeading, yaw);

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
        double rate = velocities.xRotationRate;

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

    public void moveRobotVelocity(double axialV, double yawV, double lateralV) {
        setAxialVelocity(axialV);
        setYawVelocity(yawV);
        moveRobotVelocity();
    }

    public void moveRobotVelocity() {
        //calculate required motor speeds
        double leftVel = driveAxial - driveYaw;
        double rightVel = driveAxial + driveYaw;

        double biggest = Math.max(Math.abs(leftVel), Math.abs(rightVel));

        if (biggest > MAX_VELOCITY) {
            double scale = MAX_VELOCITY / biggest;
            leftVel *= scale;
            rightVel *= scale;
        }

        leftDrive.setVelocity(leftVel);
        rightDrive.setVelocity(rightVel);

        myOpMode.telemetry.addData("Motor Vel (CPS)", "%3.1f %3.1f", leftVel, rightVel);


        // Log.d("G-FORCE AUTO", String.format("M %5.1f %5.1f %5.1f %5.1f ", leftVel, rightVel, leftBackVel, rightBackVel));
    }

    public void setAxialPower(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void stopRobot() {
        moveRobotVelocity(0, 0, 0);
    }

    // ========================================================
    // ----               Motor Methods
    // ========================================================

    public void runCollectors(double power) {
        frontCollector.setPower(power);
        midCollector.setPower(power);
    }

    public void runSpinners(double spinnerSpeed) {
        targetSpinnerSpeed = spinnerSpeed;
        leftShooter.setVelocity(spinnerSpeed);
        rightShooter.setVelocity(spinnerSpeed);
    }

    public boolean spinnerAtSpeed(double spinnerSpeed) {
        runSpinners(spinnerSpeed);
        return (Math.abs(targetSpinnerSpeed - leftShooter.getVelocity()) < SHOOTER_SPEED_TEST);
    }

    // ========================================================
    // ----               SERVO Methods
    // ========================================================

    public void releaseWobbleGoal() {
        leftWobbleGrab.setPosition(LEFT_GOAL_RELEASE);
        rightWobbleGrab.setPosition(RIGHT_GOAL_RELEASE);
    }

    public void grabWobbleGoal() {
        leftWobbleGrab.setPosition(LEFT_GOAL_GRAB);
        rightWobbleGrab.setPosition(RIGHT_GOAL_GRAB);
    }

    public void stopRings() {
        ringStop.setPosition(RING_STOP);
    }

    public void releaseRings() {
        ringStop.setPosition(RING_RELEASE);
    }

}