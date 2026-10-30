package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

/**
 * Steerable shooter: turret (motor + encoder), hood (servo) and a single flywheel motor.
 *
 * Aiming uses Pedro odometry and a known hive-Cell location.  While TRACKING, every scheduler
 * loop (periodic) computes a firing solution:
 *   - Shoot on the move: the robot's field velocity is carried by the game piece, so we aim at a
 *     "virtual target" = target - robotVelocity * flightTime (iterated a few times).
 *   - Distance to the virtual target selects flywheel RPM, hood position and flight time from
 *     an interpolated lookup table (SHOT_TABLE).
 *   - The turret angle (relative to robot forward, + is CCW) points at the virtual target, with a
 *     feedforward that counter-rotates the turret while the chassis turns.
 *
 * The turret encoder is zeroed in init(), so the turret MUST be centered (pointing robot-forward)
 * when the opmode is initialized.
 *
 * Intake and transfer (feeding balls) are handled by IntakeSubsystem.
 */
public class ShooterSubsystem extends SubsystemBase {

    // == Field targets ========================================================
    // Goal locations in Pedro coordinates (inches, origin at a field corner, 0-144).
    // RED side is x = 60, BLUE side x = 84.  Blue is red with y flipped about 72.
    // Each alliance has two goals; we aim at the nearer one.
    public static final Pose RED_GOAL_AUDIENCE  = new Pose(60, 54.5);
    public static final Pose RED_GOAL_FAR       = new Pose(60, 144 - 54.5);

    // Only switch to the other alliance goal when it is this much closer (inches), so the
    // target doesn't flicker when the robot is about equidistant.
    private static final double GOAL_SWITCH_HYSTERESIS = 6.0;

    // Turret pivot location relative to robot center (inches, x = forward, y = left).
    private static final double TURRET_OFFSET_X = 0.0;
    private static final double TURRET_OFFSET_Y = 0.0;

    // == Turret ==============================================================
    // false: the turret is held at 0 (straight ahead) and the chassis turns to aim instead
    // (teleop point-at-goal mode).  isReady() then checks the robot heading.
    public static final boolean TURRET_ENABLED        = true;
    public static final boolean TURRET_SIMULATED      = false;
    private static final double AIM_TOLERANCE_RAD     = Math.toRadians(10);

    // Encoder ticks per radian of turret rotation: 537.7 CPR motor, direct drive (no gear ratio).
    private static final double TURRET_TICKS_PER_RAD  = 537.7 / (2 * Math.PI);
    private static final double TURRET_MIN_RAD        = Math.toRadians(-170);
    private static final double TURRET_MAX_RAD        = Math.toRadians(170);
    private static final double TURRET_KP             = 0.5;    // power per radian of error
    private static final double TURRET_KV             = 0.15;   // power per rad/s of chassis turn
    private static final double TURRET_MAX_POWER      = 0.8;
    private static final double TURRET_TOLERANCE_RAD  = Math.toRadians(5);

    // == Flywheel ============================================================
    // Ticks per rev of the flywheel (28 for a bare goBILDA/REV motor at 1:1).  Check with
    // util/FlywheelTestOpMode: full power should read close to the motor's free speed.
    public static final double FLYWHEEL_TICKS_PER_REV = 28;

    public static double rpmToTicksPerSec(double rpm) { return rpm * FLYWHEEL_TICKS_PER_REV / 60.0; }
    public static double ticksPerSecToRpm(double tps) { return tps * 60.0 / FLYWHEEL_TICKS_PER_REV; }
    private static final double FLYWHEEL_MAX_RPM       = 5500;
    private static final double RPM_TOLERANCE          = 400;
    private static final double FLYWHEEL_IDLE_POWER    = 0.0;

    // == Hood ================================================================
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 1.0;

    // == Shot table ==========================================================
    // TODO placeholder: fill with measurements from ShooterTuningOpMode.
    // Rows sorted by distance: { distance (in), flywheel RPM, hood servo position, flight time (s) }
    private static final double[][] SHOT_TABLE = {
        {  24, 4400, 0.20, 0.30 },
        {  48, 4400, 0.35, 0.40 },
        {  72, 4400, 0.50, 0.50 },
        {  96, 4400, 0.60, 0.60 },
        { 120, 4400, 0.70, 0.70 },
    };

    // Number of iterations used to converge the shoot-on-the-move solution.
    private static final int LEAD_ITERATIONS = 3;

    public enum Mode { IDLE, MANUAL, TRACKING }

    private final OpMode   opMode;
    private final Follower drive;
    private DcMotorEx turret;
    private DcMotorEx flywheel;
    private Servo     hood;

    private Mode    mode    = Mode.IDLE;
    private boolean redAlliance = true;
    private boolean farGoal     = true;     // which of the alliance's two goals is targeted
    private double  dFar        = 0;        // robot distance to each alliance goal (inches)
    private double  dAudience   = 0;

    // FULL_SPEED spin-up
    private final ElapsedTime spinupTimer = new ElapsedTime();
    private boolean           wasSpinning = false;

    // Setpoints
    private double targetRPM        = 0;
    private double targetHood       = HOOD_MIN;
    private double targetTurretRad  = 0;
    private double actualRPM        = 0;
    private double actualTurretRad  = 0;

    // Latest solution (also computed when not tracking, for telemetry and tuning)
    private double distance         = 0;
    private Shot   shot             = new Shot(0, 0, 0, HOOD_MIN);
    private double turretX          = 0;    // turret pivot, field coordinates
    private double turretY          = 0;

    // == standard Methods ===================================
    public ShooterSubsystem(OpMode myOpMode, Follower drive) {
        opMode   = myOpMode;
        this.drive = drive;
    }

    public void init() {
        turret   = opMode.hardwareMap.get(DcMotorEx.class, "turret");
        flywheel = opMode.hardwareMap.get(DcMotorEx.class, "shooter");
        hood     = opMode.hardwareMap.get(Servo.class, "hood");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(3,0.5,0,10.0));

        // Move the hood to 0 now, rather than waiting for the first periodic() after start.
        targetHood = 0;
        hood.setPosition(targetHood);

        idle();
    }

    @Override
    public void periodic() {

        actualRPM        = ticksPerSecToRpm(flywheel.getVelocity());
        actualTurretRad  = turret.getCurrentPosition() / TURRET_TICKS_PER_RAD;

        computeSolution();

        if (mode == Mode.TRACKING) {
            targetRPM       = shot.rpm;
            targetHood      = shot.hood;
            targetTurretRad = TURRET_ENABLED ? turretAngleTo(shot.aimX, shot.aimY) : 0;
        }

        applyOutputs();
        showTelemetry();
    }

    void showTelemetry() {
        opMode.telemetry.addData("SHOOTER", "%s %s %s  dist = %5.1f  ready = %b",
                mode, redAlliance ? "RED" : "BLUE", farGoal ? "FAR" : "AUDIENCE", distance, isReady());
        opMode.telemetry.addData("  Goal dist audience : far", "%5.1f : %5.1f", dAudience, dFar);
        opMode.telemetry.addData("  RPM tgt : act", "%5.0f : %5.0f", targetRPM, getActualRPM());
        opMode.telemetry.addData("  Turret tgt : act", "%5.1f : %5.1f",
                Math.toDegrees(targetTurretRad), Math.toDegrees(getactualTurretRad()));

        // Where the turret should point to hit the cell, computed every loop even when not
        // tracking.  "robot" is relative to robot forward (+ CCW, before soft limits);
        // "field" is the Pedro field direction from the turret to the aim point.
        opMode.telemetry.addData("  Aim robot : field", "%5.1f : %5.1f",
                Math.toDegrees(getAimError()), Math.toDegrees(getAimHeading()));
        opMode.telemetry.addData("  Hood", "%4.2f", targetHood);
    }

    // == new elements =========================================

    public void setAlliance(boolean red) {
        redAlliance = red;
    }

    /** Aim at the alliance hive-Cell and spin up, every loop. */
    public void track() {
        mode = Mode.TRACKING;
    }

    /** Stop the flywheel and hold the turret where it is pointing. */
    public void idle() {
        mode = Mode.IDLE;
        targetRPM = 0;
        if (turret != null) {
            targetTurretRad = getactualTurretRad();
        }
    }

    /** Fixed shot, for tuning and as a fallback if odometry is lost. */
    public void setManual(double rpm, double hoodPos, double turretRad) {
        mode            = Mode.MANUAL;
        targetRPM       = rpm;
        targetHood      = hoodPos;
        targetTurretRad = turretRad;
    }

    /** True when the flywheel is at speed and the turret (or the robot, if no turret) is on target. */
    public boolean isReady() {
        boolean onTarget = TURRET_ENABLED
                ? Math.abs(targetTurretRad - getactualTurretRad()) < TURRET_TOLERANCE_RAD && turretTargetReachable()
                : mode != Mode.TRACKING || Math.abs(getAimError()) < AIM_TOLERANCE_RAD;
        boolean atSpeed = Math.abs(getActualRPM() - targetRPM) < RPM_TOLERANCE;
        return mode != Mode.IDLE
                && targetRPM > 0
                && atSpeed
                && onTarget;
    }

    /** Pedro field direction (radians) from the turret to the lead-corrected aim point. */
    public double getAimHeading() {
        return Math.atan2(shot.aimY - turretY, shot.aimX - turretX);
    }

    /** How far to turn the robot (radians, + is CCW) to point straight at the aim point. */
    public double getAimError() { return Angle.normalizeSigned(getAimHeading() - drive.pose().heading());
    }

    /** Distance (inches) from the turret to the (virtual) target. */
    public double getDistance() {
        return distance;
    }

    public Mode getMode() {
        return mode;
    }

    /** Turret angle relative to robot forward, radians, + is CCW. */

    public double getActualRPM() {
        if (TURRET_SIMULATED) {
            return targetRPM;
        } else {
            return actualRPM;
        }
    }

    public double getactualTurretRad() {
        if (TURRET_SIMULATED) {
            return targetTurretRad;
        } else {
            return turret.getCurrentPosition() / TURRET_TICKS_PER_RAD;
        }
    }

    public double getTargetRPM()       { return targetRPM; }
    public double getTargetHood()      { return targetHood; }
    public double getTargetTurretRad() { return targetTurretRad; }

    // == Aiming math ===========================================

    /** The alliance goal we're aiming at: the nearer one, with hysteresis. */
    private Pose target() {
        Pose far      =  RED_GOAL_FAR;
        Pose audience =  RED_GOAL_AUDIENCE;

        Pose robot = drive.pose();
        dFar      = robot.distance(far);
        dAudience = robot.distance(audience);
        if (farGoal && dAudience < dFar - GOAL_SWITCH_HYSTERESIS) {
            farGoal = false;
        } else if (!farGoal && dFar < dAudience - GOAL_SWITCH_HYSTERESIS) {
            farGoal = true;
        }
        return farGoal ? far : audience;
    }

    /** Computes the shoot-on-the-move firing solution from odometry. */
    private void computeSolution() {
        Pose     pose    = drive.pose();
        Velocity vel     = drive.velocity();   // field frame, inches/s
        double   heading = pose.heading();

        // Turret pivot in field coordinates.
        double tx = pose.x() + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double ty = pose.y() + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        Pose   cell = target();
        double aimX = cell.x();
        double aimY = cell.y();
        double tof  = 0;
        double dist = 0;
        double[] row = null;

        for (int i = 0; i < LEAD_ITERATIONS; i++) {
            aimX = cell.x() - vel.vx * tof;
            aimY = cell.y() - vel.vy * tof;
            dist = Math.hypot(aimX - tx, aimY - ty);
            row  = interpolate(dist);
            tof  = row[3];
        }

        distance = dist;
        shot = new Shot(aimX, aimY, row[1], row[2]);
        turretX = tx;
        turretY = ty;
    }

    /** Turret angle (relative to robot forward) that points at a field location. */
    private double turretAngleTo(double x, double y) {
        double fieldAngle = Math.atan2(y - turretY, x - turretX);
        double angle = Angle.normalizeSigned(fieldAngle - drive.pose().heading());
        return Range.clip(angle, TURRET_MIN_RAD, TURRET_MAX_RAD);
    }

    /** False if the aim point is outside the turret's travel (clipped at a soft limit). */
    private boolean turretTargetReachable() {
        if (mode != Mode.TRACKING) return true;
        double fieldAngle = Math.atan2(shot.aimY - turretY, shot.aimX - turretX);
        double angle = Angle.normalizeSigned(fieldAngle - drive.pose().heading());
        return angle >= TURRET_MIN_RAD && angle <= TURRET_MAX_RAD;
    }

    /** Linear interpolation of SHOT_TABLE by distance, clamped to the end rows. */
    static double[] interpolate(double dist) {
        if (dist <= SHOT_TABLE[0][0]) return SHOT_TABLE[0];
        int last = SHOT_TABLE.length - 1;
        if (dist >= SHOT_TABLE[last][0]) return SHOT_TABLE[last];

        for (int i = 1; i <= last; i++) {
            double[] hi = SHOT_TABLE[i];
            if (dist <= hi[0]) {
                double[] lo = SHOT_TABLE[i - 1];
                double t = (dist - lo[0]) / (hi[0] - lo[0]);
                double[] out = new double[lo.length];
                for (int c = 0; c < lo.length; c++) {
                    out[c] = lo[c] + t * (hi[c] - lo[c]);
                }
                return out;
            }
        }
        return SHOT_TABLE[last];
    }

    private void applyOutputs() {

        // Control Turret
        // Turret: P on angle + feedforward to cancel chassis rotation.
        double error = targetTurretRad - getactualTurretRad();
        double power = error * TURRET_KP;
        if (mode == Mode.TRACKING && TURRET_ENABLED) {
            power -= drive.velocity().omega * TURRET_KV;
        }
        // Never drive past the soft limits.
        double angle = getactualTurretRad();
        if ((angle >= TURRET_MAX_RAD && power > 0) || (angle <= TURRET_MIN_RAD && power < 0)) {
            power = 0;
        }
        turret.setPower(Range.clip(power, -TURRET_MAX_POWER, TURRET_MAX_POWER));

        // Control Flywheel Switch between full power and calculated power.
        if (targetRPM > 0) {
            flywheel.setVelocity(rpmToTicksPerSec(targetRPM));   // setVelocity takes ticks/s
        } else {
            flywheel.setPower(FLYWHEEL_IDLE_POWER);
        }

        // Control Hood Angle
        hood.setPosition(Range.clip(targetHood, HOOD_MIN, HOOD_MAX));
    }

    private static class Shot {
        final double aimX, aimY, rpm, hood;
        Shot(double aimX, double aimY, double rpm, double hood) {
            this.aimX = aimX;
            this.aimY = aimY;
            this.rpm  = rpm;
            this.hood = hood;
        }
    }

    // == Command methods  ==========================================

    /** Track while scheduled (e.g. while a button is held); go idle when it ends. */
    public Command trackCommand() {
        return new StartEndCommand(this::track, this::idle, this);
    }

    /** Start tracking and leave it running (for auto sequences). */
    public Command startTrackingCommand() {
        return new InstantCommand(this::track, this);
    }

    public Command stopTrackingCommand() {
        return new InstantCommand(this::idle, this);
    }

    public Command idleCommand() {
        return new InstantCommand(this::idle, this);
    }

    public Command manualCommand(double rpm, double hoodPos, double turretRad) {
        return new InstantCommand(() -> setManual(rpm, hoodPos, turretRad), this);
    }

    /** Finishes when ready to shoot, or after timeoutMs. Use before the transfer's feed command. */
    public Command waitUntilReadyCommand(long timeoutMs) {
        return new WaitUntilCommand(this::isReady).withTimeout(timeoutMs);
    }
}
