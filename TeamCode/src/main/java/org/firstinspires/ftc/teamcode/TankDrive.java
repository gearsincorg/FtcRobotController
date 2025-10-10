package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.digitalchickenlabs.OctoQuad.I2cRecoveryMode.MODE_2_M1_PLUS_SCL_IDLE_ONESHOT_TGL;

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
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.messages.TankCommandMessage;
import org.firstinspires.ftc.teamcode.messages.TankLocalizerInputsMessage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

@Config
public final class TankDrive {
    OctoQuad.LocalizerDataBlock OQlocalizer = new OctoQuad.LocalizerDataBlock();

    public static class Params {

        OctoQuad.LocalizerDataBlock OQlocalizer = new OctoQuad.LocalizerDataBlock();

        // drive model parameters
        public double inPerTick = 0;
        public double trackWidthTicks = 0;

        // feedforward parameters (in tick units)
        public double kS = 0.300;
        public double kV = 0.006;
        public double kA = 0.001;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -50;
        public double maxProfileAccel = 150;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI * 2;

        // path controller gains
        public double ramseteZeta = 0.7; // in the range (0, 1)
        public double ramseteBBar = 2.0; // positive

        // turn controller gains
        public double turnGain = 6.0;
        public double turnVelGain = 0.0;
    }

    public static Params PARAMS = new Params();

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

    public final List<DcMotorEx> leftMotors, rightMotors;
    public final LinearOpMode myOpMode;
    public final VoltageSensor voltageSensor;
    public final OctoQuad oq;
    private Pose2d pose;

    //public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    public TankDrive(HardwareMap hardwareMap, Pose2d pose, LinearOpMode opMode) {
        myOpMode = opMode;
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   add additional motors on each side if you have them
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "left_drive"));
        rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "right_drive"));

        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        oq = hardwareMap.get(OctoQuad.class, "octoquad");
        intializeOctoQuad(oq);
        setPose(pose);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        FlightRecorder.write("TANK_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
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
    }

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

    public PoseVelocity2d updatePoseEstimate() {

        oq.readLocalizerData(OQlocalizer);
        if (OQlocalizer.isDataValid()) {

            myOpMode.telemetry.addData("X:Y:H in,Deg", "%4.1f %4.1f %4.0f",
                    mmToInch(OQlocalizer.posX_mm), mmToInch(OQlocalizer.posY_mm), Math.toDegrees(OQlocalizer.heading_rad));

            pose = new Pose2d(mmToInch(OQlocalizer.posX_mm), mmToInch(OQlocalizer.posY_mm), OQlocalizer.heading_rad);

            return new PoseVelocity2d(new Vector2d(OQlocalizer.velX_mmS, OQlocalizer.velY_mmS), OQlocalizer.velHeading_radS);
        } else {
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }
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

    private void intializeOctoQuad(OctoQuad oq) {

        // X_OFFSET_FROM_CENTER_MM is how sideways from the center of the robot is the X (forward) pod? Left increases
        // Y_OFFSET_FROM_CENTER_MM is how far forward from the center of the robot is the Y (Strafe) pod? forward increases
        final float TICKS_PER_MM = 19.89f;
        final float X_OFFSET_FROM_CENTER_MM = -27.0f;
        final float Y_OFFSET_FROM_CENTER_MM =  25.8f;
        final float OQ_IMU_SCALAR = (float)(360.0/343.4);
        final int OQ_PORT_X = 0;
        final int OQ_PORT_Y = 1;

        oq.resetEverything();

        oq.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);


        // Configure the localizer
        oq.setSingleEncoderDirection(OQ_PORT_X, OctoQuad.EncoderDirection.FORWARD);
        oq.setSingleEncoderDirection(OQ_PORT_Y, OctoQuad.EncoderDirection.FORWARD);

        oq.setLocalizerPortX(OQ_PORT_X);
        oq.setLocalizerPortY(OQ_PORT_Y);
        oq.setLocalizerCountsPerMM_X(TICKS_PER_MM);
        oq.setLocalizerCountsPerMM_Y(TICKS_PER_MM);
        oq.setLocalizerTcpOffsetMM_X(X_OFFSET_FROM_CENTER_MM);
        oq.setLocalizerTcpOffsetMM_Y(Y_OFFSET_FROM_CENTER_MM);
        oq.setLocalizerImuHeadingScalar(OQ_IMU_SCALAR);
        oq.setLocalizerVelocityIntervalMS(25);

        oq.setI2cRecoveryMode(MODE_2_M1_PLUS_SCL_IDLE_ONESHOT_TGL);
        /*if (Globals.IS_AUTO) {
            oq.saveParametersToFlash();
        }*/
        oq.resetLocalizerAndCalibrateIMU();
    }

    public double getTurnRateDPS() {
        return Math.toDegrees(OQlocalizer.velHeading_radS);
    }

    public void setPose (Pose2d newPose) {
        oq.setLocalizerPose(inchToMm(newPose.position.x), inchToMm(newPose.position.y), (float)newPose.heading.toDouble());
        pose = newPose;
    }

    public Pose2d getPose () {
        return pose;
    }

    public double getHeadingRad() {
        return pose.heading.toDouble();
    }

    public double getHeadingDeg() {
        return Math.toDegrees(pose.heading.toDouble());
    }

    private double mmToInch(double mm) {
        return mm / 25.4;
    }

    private int inchToMm(double inches) {
        return (int)(inches * 25.4);
    }
}
