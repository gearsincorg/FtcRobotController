package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.auxtools.Drawing;
import org.firstinspires.ftc.teamcode.auxtools.SharedOQ;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.messages.TankCommandMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class DriveSubsystem
{
    private boolean showTelemetry = false;
    private static boolean enabled = false;
    private LinearOpMode myOpMode;

    public  static OctoQuad oq = null;

    private boolean     intakeForward = false;
    private boolean     intakeReverse = true;
    private boolean     headingLocked = false;
    private double      headingSetpointDeg = 0;

    public static class Params {
        // drive model parameters (Tick is 1 mm)
        public double inPerTick       = 0.0392;  // approx
        public double trackWidthTicks = 350;     // approx

        // feedforward parameters (in tick (mm) units)
        public double kS = 0.00;  // was 0.3
        public double kV = 0.006;  // was 0.006
        public double kA = 0.00;

        // path profile parameters (in inches)
        public double maxWheelVel     =  40;
        public double minProfileAccel = -50;
        public double maxProfileAccel = 100;

        // turn profile parameters (in radians)
        public double maxAngVel   = Math.PI; // shared with path
        public double maxAngAccel = Math.PI * 4;

        // path controller gains
        public double ramseteBBar = 2.0; // Like P gain (was 2)
        public double ramseteZeta = 0.8; // in the range (0, 1)  was 0.7

        // turn controller gains
        public double turnGain    = 4.0;  // was 4
        public double turnVelGain = 0.0;  // was 0
    }

    public static Params PARAMS = new Params();

    static final double DRIVE_DEADBAND      =  0.1;      // Dont start turning until we need to move position.
    static final double TURN_DEADBAND       =  0.1;      // Lock heading if JoyStick less than this
    static final double HEADING_GAIN        =  0.015;    // turn at full power of the error
    static final double HEADING_TOLLERANCE  = 30.0;      // Don't start driving until we are withing 30 Degrees

    static final double FC_SPEED_SCALE      =  1.0;      // Safe FC speed
    static final double RC_SPEED_SCALE      =  1.0;      // Safe RC Speed
    static final double RC_TURN_SCALE       =  0.5;      // Safe RC Turn

    static final double INTAKE_HYSTERESIS   = 0.1 ;
    static final double MIN_ROTATE          = 1.0 ;
    static final double RAD2DEG             = 180/Math.PI;
    static final double INCH2MM             = 2.54;


    public final TankKinematics kinematics = new TankKinematics(PARAMS.inPerTick * PARAMS.trackWidthTicks);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public  List<DcMotorEx> leftMotors, rightMotors;
    public  VoltageSensor voltageSensor;

    private Pose2d pose;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    public DriveSubsystem(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init(Pose2d pose, boolean showTelemetry) {
        this.showTelemetry = showTelemetry;
        this.enabled = true;

        LynxFirmware.throwIfModulesAreOutdated(myOpMode.hardwareMap);

        for (LynxModule module : myOpMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   add additional motors on each side if you have them
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftMotors  = Arrays.asList(myOpMode.hardwareMap.get(DcMotorEx.class, "front_left_drive"),
                                    myOpMode.hardwareMap.get(DcMotorEx.class, "back_left_drive"));
        rightMotors = Arrays.asList(myOpMode.hardwareMap.get(DcMotorEx.class, "front_right_drive"),
                                    myOpMode.hardwareMap.get(DcMotorEx.class, "back_right_drive"));

        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        SharedOQ.init(myOpMode);
        setPose(pose);

        voltageSensor = myOpMode.hardwareMap.voltageSensor.iterator().next();

        FlightRecorder.write("TANK_PARAMS", PARAMS);
    }

    /**
     * Choose between Field Centric and Robot Centric based on Joystick priorit
     */
    public void smartDrive() {
        double jsLeftX  = -myOpMode.gamepad1.left_stick_x;
        double jsLeftY  = -myOpMode.gamepad1.left_stick_y;
        double jsRightX = -myOpMode.gamepad1.right_stick_x;
        double jsRightY = -myOpMode.gamepad1.right_stick_y;

        // Assume Field Centric Driving takes precidence   (could be changed)
        if (Math.hypot(jsLeftX, jsLeftY) > DRIVE_DEADBAND) {
            // Field Centric driving
            driveFC(jsLeftY * FC_SPEED_SCALE, jsLeftX * FC_SPEED_SCALE);
        } else {
            // Robot Centric driving (with joystick squaring)
            driveRC(lessSensitive(jsRightY) * RC_SPEED_SCALE, lessSensitive(jsRightX) * RC_TURN_SCALE);
        }
    }

    /**
     * Apply Drive and Turn as requested by 2 joystick values
     * @param fwd           Power for moving forwared
     * @param rotateCCW     Power for turning CCW
     */
    public void driveRC (double fwd, double rotateCCW) {

        // lock the heading if we are not turning manually
        if (Math.abs(rotateCCW) < TURN_DEADBAND ) {
            if (headingLocked) {
                rotateCCW = normalizeAngle(headingSetpointDeg - getHeadingDeg()) * HEADING_GAIN;
            } else if (getTurnRateDPS() < MIN_ROTATE) {
                headingSetpointDeg = getHeadingDeg();
                headingLocked = true;
            }
        } else {
            headingLocked = false;
        }

        // send axis powers to drive
        setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, 0), rotateCCW));
    }

    /**
     * Turn and move in the field centric direction of joystick
     * @param fwd    Forward component of drive vector
     * @param left   Left component of drive vector
     */
    public void driveFC (double fwd, double left) {
        double commandMagnitude;
        double commandAngle;
        double error = 0;
        double turnPower = 0;
        double drivePower = 0;

        // Convert Joystick values to Polar Coordinates.
        commandAngle     = Math.atan2(left,fwd) * RAD2DEG;
        commandMagnitude = Math.hypot(fwd,left);

        headingSetpointDeg = commandAngle;  // save for Robot Centric Driving
        headingLocked = false;              // save for Robot Centric Driving

        // calculate error, and normalize at +/- 180 degrees
        error = normalizeAngle(commandAngle - getHeadingDeg());

        // see if driving backwards would be easier
        if (Math.abs(error) > 90) {
            error = normalizeAngle(error - 180);
            commandMagnitude = -commandMagnitude;
        }

        // Apply proportional Gain
        turnPower = error * HEADING_GAIN;

        // Only drive forward if we are within heading tollerance (turn then drive)
        if (Math.abs(error) < HEADING_TOLLERANCE) {
            drivePower = commandMagnitude;
        } else {
            drivePower = commandMagnitude / 2.0;
        }

        // send axis powers to drive
        driveRC(drivePower,turnPower);
    }

    /**
     * Convert any angle to a +/- 180  degree value.
     * @param angle
     * @return
     */
    double normalizeAngle(double angle){
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }

    /**
     * Convert axis motions to drive motor powers
     * @param powers
     */
    public void setDrivePowers(PoseVelocity2d powers) {

        // skip if not initialized
        if (!enabled) return;

        Globals.AXIAL_MOTION = powers.linearVel.x;

        TankKinematics.WheelVelocities<Time> wheelVels = new TankKinematics(2).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        for (DcMotorEx m : leftMotors) {
            m.setPower(wheelVels.left.get(0) / maxPowerMag);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(wheelVels.right.get(0) / maxPowerMag);
        }

        if (showTelemetry){
            myOpMode.telemetry.addData("drive L:R", "%.2f <-> %.2f ",
                                        wheelVels.left.get(0), wheelVels.right.get(0));
        }
    }

    /**
     * steer robot along pre-programmed trajectory based on time
     */
    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }

                return false;
            }

            DualNum<Time> x = timeTrajectory.profile.get(t);

            Pose2dDual<Arclength> txWorldTarget = timeTrajectory.path.get(x.value(), 3);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new RamseteController(kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(x, txWorldTarget, getPose());
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            for (DcMotorEx m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(rightPower);
            }

            p.put("x", getPose().position.x);
            p.put("y", getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    /**
     * perform a turn during Auto sequences.
     */
    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3),
                    txWorldTarget.heading.velocity().plus(
                            PARAMS.turnGain * getPose().heading.minus(txWorldTarget.heading.value()) +
                            PARAMS.turnVelGain * (robotVelRobot.angVel - txWorldTarget.heading.velocity().value())
                    )
            );
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            for (DcMotorEx m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(rightPower);
            }

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    /**
     * Use localization from Octoquad to update robot position.
     * @return
     */


    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d poseVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

        if (oq != null) {
            // Read localizer data AND encoder.  Process each if they are valid.
            SharedOQ.update();

            if (SharedOQ.OQlocalizer.isDataValid()) {
                myOpMode.telemetry.addData("X:Y:H inch,Deg", "%4.1f  %4.1f  %4.0f",
                        mmToInch(SharedOQ.OQlocalizer.posX_mm), mmToInch(SharedOQ.OQlocalizer.posY_mm), Math.toDegrees(SharedOQ.OQlocalizer.heading_rad));

                pose = new Pose2d(mmToInch(SharedOQ.OQlocalizer.posX_mm), mmToInch(SharedOQ.OQlocalizer.posY_mm), SharedOQ.OQlocalizer.heading_rad);
                Globals.LAST_POSE = pose ;

                poseVel = new PoseVelocity2d(new Vector2d(mmToInch(SharedOQ.OQlocalizer.velX_mmS), mmToInch(SharedOQ.OQlocalizer.velY_mmS)),
                        SharedOQ.OQlocalizer.velHeading_radS);
            }
        }

        return poseVel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public double getTurnRateDPS() {
        return Math.toDegrees(SharedOQ.OQlocalizer.velHeading_radS);
    }

    public void setPose (Pose2d newPose) {
        pose = newPose;
        if (oq != null){
            oq.setLocalizerPose(inchToMm(newPose.position.x), inchToMm(newPose.position.y), (float) newPose.heading.toDouble());
        }
    }

    public void setHeadingDeg(double heading) {
        setPose(new Pose2d(getPose().position.x, getPose().position.y, Math.toRadians(heading)));
    }

    public Pose2d getPose () {
        return pose;
    }

    public double getHeadingRad() {
        return pose.heading.toDouble();
    }

    public double getHeadingDeg() {
        return Math.toDegrees(getHeadingRad());
    }

    public static boolean isEnabled() {
        return enabled;
    }

    private double mmToInch(double mm) {
        return mm / 25.4;
    }

    private int inchToMm(double inches) {
        return (int)(inches * 25.4);
    }

    private double lessSensitive(double joystick) {
        return (joystick * joystick * Math.signum(joystick));
    }


}
