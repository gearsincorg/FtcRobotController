package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

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
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auxtools.SharedOQ;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class DriveSubsystem
{
    private boolean showTelemetry = false;
    private static boolean enabled = false;
    private LinearOpMode myOpMode;

    private boolean     headingLocked = false;
    private double      headingSetpointDeg = 0;

    private ElapsedTime actionTime      = new ElapsedTime();
    private boolean isTiming            = false;


    public static class Params {
        // drive model parameters (Tick is 1 mm)
        public double inPerTick       = 0.0397;  // approx
        public double trackWidthTicks = 368;     // approx

        // feedforward parameters (in tick (mm) units)
        public double kS = 0.00;  // was 0.3
        public double kV = 0.006;  // was 0.006
        public double kA = 0.00;

        // path profile parameters (in inches)
        public double maxWheelVel     =  50;
        public double minProfileAccel = -60;
        public double maxProfileAccel =  80;

        // turn profile parameters (in radians)
        public double maxAngVel   = Math.PI / 1.5; // shared with path was:   PI / 2
        public double maxAngAccel = Math.PI * 1.5;  // was PI*2

        // path controller gains
        public double ramseteBBar = 2.0; // Like P gain.. Strength of convergence to real path
        public double ramseteZeta = 0.8; // in the range (0, 1)  was 0.7
    }

    public static Params PARAMS = new Params();

    static final double TURN_DEADBAND       =  0.05;      // Lock heading if JoyStick less than this
    static final double HEADING_GAIN        =  0.01;    // turn at full power of the error was .015

    static final double RC_SPEED_SCALE      =  1.0;      // Safe RC Speed
    static final double RC_TURN_SCALE       =  0.5;      // Safe RC Turn

    static final double INTAKE_HYSTERESIS   = 0.1 ;
    static final double MIN_ROTATE          = 1.0 ;

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

    private Pose2d pose = new Pose2d(0,0,0);
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

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
        if (pose != null) {
            setPose(pose);
        }

        voltageSensor = myOpMode.hardwareMap.voltageSensor.iterator().next();
        FlightRecorder.write("TANK_PARAMS", PARAMS);

        Globals.FORWARD_MOTION = true;
    }

    public void smartDrive() {
        double jsLeftY  = -myOpMode.gamepad1.left_stick_y;
        double jsRightX = -myOpMode.gamepad1.right_stick_x;

        // Robot Centric driving (with joystick squaring)
        driveRC(lessSensitive(jsLeftY) * RC_SPEED_SCALE, lessSensitive(jsRightX) * RC_TURN_SCALE);
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
            } else if (Math.abs(getTurnRateDPS()) < MIN_ROTATE) {
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
     * Convert any angle to a +/- 180  degree value.
     * @param angle
     * @return
     */
    private double normalizeAngle(double angle){
        while (angle > 180) { angle -= 360; }
        while (angle < -180) { angle += 360;  }
        return angle;
    }

    /**
     * Convert axis motions to drive motor powers
     * @param powers
     */
    public void setDrivePowers(PoseVelocity2d powers) {

        // skip if not initialized
        if (!enabled) return;

        if (Globals.FORWARD_MOTION && (powers.linearVel.x < -INTAKE_HYSTERESIS)){
            Globals.FORWARD_MOTION = false;
        } else if (!Globals.FORWARD_MOTION && (powers.linearVel.x > INTAKE_HYSTERESIS)){
            Globals.FORWARD_MOTION = true;
        }

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
            // myOpMode.telemetry.addData("MOTION", "%s", Globals.FORWARD_MOTION ? "FORWARD" : "REVERSE");
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
            updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new RamseteController(kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(x, txWorldTarget, getPose());

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;

            for (DcMotorEx m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(rightPower);
            }
            return true;
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
            updatePoseEstimate();

            myOpMode.telemetry.addData("TURN", "TA= %.1f, TV= %.1f", txWorldTarget.heading.value().toDouble(),txWorldTarget.heading.velocity().value());

            double rotateCCW = normalizeAngle(Math.toDegrees(txWorldTarget.heading.value().toDouble()) - getHeadingDeg()) * HEADING_GAIN;

            // send axis powers to drive
            setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), rotateCCW));

            return true;
        }
    }

    /**
     * Use localization from Octoquad to update robot position.
     * @return
     */
    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d poseVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

        // Read localizer data AND encoder.  Process each if they are valid.
        SharedOQ.update();
        if (SharedOQ.OQlocalizer.isDataValid()) {
            myOpMode.telemetry.addData("LOCATION", "X=%5.1f  Y=%5.1f  H=%4.0f\n",
                    mmToInch(SharedOQ.OQlocalizer.posX_mm), mmToInch(SharedOQ.OQlocalizer.posY_mm),
                    Math.toDegrees(SharedOQ.OQlocalizer.heading_rad));

            // myOpMode.telemetry.addData("VELOCITY", "L:%5.2f IPS A:%5.2f RPS",
            // mmToInch(Math.hypot(SharedOQ.OQlocalizer.velX_mmS, SharedOQ.OQlocalizer.velY_mmS)), SharedOQ.OQlocalizer.velHeading_radS);

            if (Globals.IS_AUTO && myOpMode.opModeIsActive()) {
                myOpMode.telemetry.update();
            }

            pose = new Pose2d(mmToInch(SharedOQ.OQlocalizer.posX_mm), mmToInch(SharedOQ.OQlocalizer.posY_mm), SharedOQ.OQlocalizer.heading_rad);
            Globals.LAST_POSE = pose ;

            poseVel = new PoseVelocity2d(new Vector2d(mmToInch(SharedOQ.OQlocalizer.velX_mmS), mmToInch(SharedOQ.OQlocalizer.velY_mmS)),
                    SharedOQ.OQlocalizer.velHeading_radS);
        }

        return poseVel;
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
        SharedOQ.setLocalizerPose(pose);
        headingSetpointDeg = Math.toDegrees(pose.heading.toDouble());
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

    private double mmToInch(double mm) {
        return mm / 25.4;
    }

    private double lessSensitive(double joystick) {
        return (joystick * joystick * Math.signum(joystick));
    }
}
