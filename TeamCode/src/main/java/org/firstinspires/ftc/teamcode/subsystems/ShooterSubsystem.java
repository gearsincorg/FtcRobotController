package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import java.util.Locale;
import java.util.function.BooleanSupplier;

/**
 * Steerable shooter: turret (motor + encoder), hood (servo) and a single flywheel motor.
 *
 * Aiming uses Pedro odometry and a known hive-Cell location.  While TRACKING, every scheduler
 * loop (periodic) computes a firing solution:
 *   - Shoot on the move: the robot's field velocity is carried by the game piece, so we aim at a
 *     "virtual target" = target - robotVelocity * flightTime (iterated a few times).
 *   - Distance to the virtual target selects flywheel RPM, hood angle and flight time from
 *     an interpolated lookup table (SHOT_TABLE).
 *   - The turret angle (from its zero, + is CCW) points at the virtual target, with a
 *     feedforward that counter-rotates the turret while the chassis turns.
 *
 * The turret encoder is zeroed in init(), so the turret MUST be centered (at its zero, where the
 * shooter points TURRET_ZERO_RAD from robot forward: straight backwards) when the opmode is
 * initialized.  Every aim (turret or chassis) allows for that offset.
 *
 * Intake and transfer (feeding balls) are handled by IntakeSubsystem.
 */
public class ShooterSubsystem extends SubsystemBase {

    // == Field targets ========================================================
    // Goal locations in Pedro coordinates (inches, origin at a field corner, 0-144).
    // RED side is x = 60, BLUE side x = 84.  Blue is red with y flipped about 72.
    // Each alliance has two goals; we aim at the nearer one.
    public static final Pose RED_GOAL_AUDIENCE  = new Pose(59.375, 55.5);
    public static final Pose RED_GOAL_FAR       = new Pose(59.375, 88.5);

    // Only switch to the other alliance goal when it is this much closer (inches), so the
    // target doesn't flicker when the robot is about equidistant.
    private static final double GOAL_SWITCH_HYSTERESIS = 6.0;

    // Turret pivot location relative to robot center (inches, x = forward, y = left).
    private static final double TURRET_OFFSET_X = 0.0;
    private static final double TURRET_OFFSET_Y = 0.0;

    // Direction the shooter points, relative to robot forward (+ is CCW), with the turret at 0.
    // 180 deg: the shooter faces straight backwards, so the chassis aims its back at the goal.
    public static final double TURRET_ZERO_RAD = Math.toRadians(180);

    // == Mode flags ==========================================================
    // TURRET_ENABLED
    //   true : the turret aims the shooter.  The robot heading is only changed by the driver.
    //          isReady() checks the turret is on its target (and can reach it).
    //   false: the turret is held at 0 (backwards) and the chassis turns to aim the shooter
    //          (teleop point-at-goal).  isReady() checks the robot heading (if setRequireChassisAim).
    // SHOOTER_SIMULATED: the flywheel motor is still commanded as normal, but its speed is read
    //   back as the speed commanded, not from the encoder (so isAtSpeed() follows the command).
    // TURRET_SIMULATED: the turret motor and hood servo are still commanded as normal, but the turret
    //   angle is read back as the angle commanded, not from the encoder.  (The hood servo has no
    //   feedback either way.)
    // Now: the shooter works, the turret can't rotate yet, so the chassis aims and the turret is
    // simulated (its loop then never pushes against the stuck turret).
    public static final boolean TURRET_ENABLED    = false;
    public static final boolean SHOOTER_SIMULATED = false;
    public static final boolean TURRET_SIMULATED  = true;

    // == Turret ==============================================================
    private static final double AIM_TOLERANCE_RAD     = Math.toRadians(5);   // chassis aim (turret disabled)

    // Encoder ticks per radian of turret rotation: 537.7 CPR motor, direct drive (no gear ratio).
    private static final double TURRET_TICKS_PER_RAD  = 537.7 / (2 * Math.PI);
    private static final double TURRET_MIN_RAD        = Math.toRadians(-170);
    private static final double TURRET_MAX_RAD        = Math.toRadians(170);
    private static final double TURRET_KP             = 0.5;    // power per radian of error
    private static final double TURRET_KV             = 0.15;   // power per rad/s of chassis turn
    private static final double TURRET_MAX_POWER      = 0.8;
    private static final double TURRET_TOLERANCE_RAD  = Math.toRadians(5);

    // == Flywheel ============================================================
    // The encoder is on the motor; a belt drives the flywheel.  Every RPM in this code (the shot
    // table, targets, tolerances, telemetry, the calibration file) is FLYWHEEL RPM; only the
    // conversions below know about the motor.
    //   MOTOR_TICKS_PER_REV : encoder ticks per motor rev (28 for a bare goBILDA 5203 / REV motor)
    //   FLYWHEEL_GEAR_RATIO : flywheel revs per motor rev (> 1 for a speed increaser, e.g.
    //                         motor pulley 36T / flywheel pulley 24T = 1.5).  4/3 (1:1.333) since
    //                         2026-11-17.  Before that it was really 48:29 (1.655), though the
    //                         code said 1.0 and then 1.5; the calibration data is corrected for it.
    // Check with util/FlywheelTestOpMode: at full power the motor RPM it shows should be close to
    // the motor's free speed.
    public static final double MOTOR_TICKS_PER_REV  = 28;
    public static final double MOTOR_FREE_RPM       = 5500;
    public static final double FLYWHEEL_GEAR_RATIO  = 4.0 / 3.0;

    /** Flywheel RPM -> motor encoder ticks/s (what setVelocity takes). */
    public static double rpmToTicksPerSec(double flywheelRpm) {
        return flywheelToMotorRpm(flywheelRpm) * MOTOR_TICKS_PER_REV / 60.0;
    }
    /** Motor encoder ticks/s (what getVelocity returns) -> flywheel RPM. */
    public static double ticksPerSecToRpm(double tps) {
        return motorToFlywheelRpm(tps * 60.0 / MOTOR_TICKS_PER_REV);
    }
    public static double flywheelToMotorRpm(double flywheelRpm) { return flywheelRpm / FLYWHEEL_GEAR_RATIO; }
    public static double motorToFlywheelRpm(double motorRpm)    { return motorRpm * FLYWHEEL_GEAR_RATIO; }

    /** Flywheel speed with the motor at its free speed (no load); nothing can be asked above this. */
    public static final double FLYWHEEL_MAX_RPM = MOTOR_FREE_RPM * FLYWHEEL_GEAR_RATIO;
    private static final double RPM_TOLERANCE          = 200;   // isAtSpeed(): within this of the target (was 400)
    private static final double FLYWHEEL_IDLE_POWER    = 0.0;

    // Speed control: our own loop, run once per robot loop in periodic(), with the motor in
    // RUN_WITHOUT_ENCODER (the hub's velocity PIDF is not used).  We replaced the hub's loop
    // (2026-11-17) because it has no battery compensation, its P was far too weak to matter
    // (P 3 = 2.5% power for an 880 RPM error), and every boost changed the motor mode, which reset
    // its integrator, so the flywheel sagged ~880 RPM for seconds after each volley.
    //
    //   power = (KS + KV * commandRPM) / volts                     feedforward: the volts that speed
    //                                                             needs, as a fraction of the battery
    //         + KP * (commandRPM - actualRPM)                    proportional, in power per RPM
    //         + integral                                         KI * error * dt, clamped to
    //                                                             +-I_MAX_POWER; only builds within
    //                                                             I_ZONE_RPM of the command and
    //                                                             is frozen while ramping/boosting
    // clipped to 0..1 (never reverse power on the belt).  While boosting: BOOST_POWER.
    //
    // KS / KV: fit to a 1000-6000 RPM sweep on a fresh battery (2026-11-17 15:57 calibration log;
    // within 0.3 V of a straight line over the whole range; KV 0.00149 then, at ratio 1.5).  With the
    // 4/3 pulleys the load dropped: KV 0.00116 from the 2026-11-17 21:48 teleop log (7.1 V at ~4000
    // RPM with KS 2.45; only ~300 RPM of targets, so KS is unchecked: re-sweep).  The log
    // has power and volts, and `python tools/shotlog.py` prints a KS / KV fit from the steady
    // stretches: paste it in here.  The feedforward should do nearly all the work; P and I only
    // trim.  Battery volts are read every VOLTS_PERIOD_S (it costs a hub call; not in the bulk read).
    private static final double NOMINAL_VOLTS          = 12.0;
    private static final double FLYWHEEL_KS            = 2.45;    // volts (the line's offset, not a true stiction value)
    private static final double FLYWHEEL_KV            = 0.00116;  // volts per flywheel RPM
    private static final double FLYWHEEL_KP            = 0.0005;  // power per RPM of error (10% per 200 RPM)
    private static final double FLYWHEEL_KI            = 0.0003;  // power per RPM*s of error
    private static final double FLYWHEEL_I_MAX_POWER   = 0.15;    // integral limit (power)
    private static final double FLYWHEEL_I_ZONE_RPM    = 300;     // integral builds only this near the command
    private static final double MIN_VOLTS              = 8.0;     // floor for the compensation (bad reading)
    private static final double VOLTS_PERIOD_S         = 0.25;
    private static final double POWER_MIN_STEP         = 0.005;   // skip smaller power changes (saves a hub write)

    // Boost: while allowed (setBoostSupplier: shoot held or the intake full), if the speed falls
    // more than BOOST_ENTER_RPM below the target (a ball just left, or the target rose), the
    // flywheel runs at BOOST_POWER until it is at or above the target (BOOST_EXIT_RPM 0), then the
    // loop above takes over again with nothing reset.  The speed reading moves in ~64 RPM steps, so
    // BOOST_ENTER_RPM must be more than one step (at 50, one step down started it and it chattered
    // on and off every 0.2-0.3 s while full).  false = loop only (boost off).
    public static final boolean BOOST_ENABLED          = true;
    private static final double BOOST_ENTER_RPM        = 100;   // start boost this far below target
    private static final double BOOST_EXIT_RPM         = 0;     // stop once at or above the target
    private static final double BOOST_POWER            = 1.0;

    // Spin-up ramp: the flywheel is heavy, and asking for full speed from a standstill slips the
    // belt.  So when it starts from (nearly) stopped, below RAMP_START_RPM, the speed asked of the
    // loop (commandRPM) rises at RAMP_RPM_PER_S, but never more than RAMP_MAX_LEAD_RPM above the
    // measured speed (it used to run 1200 RPM ahead, which wound up the hub's integrator and
    // overshot 650 RPM), until it reaches the target; boost and the integral are off while
    // ramping.  Once running, speed changes are not ramped.  Tune RAMP_RPM_PER_S up until the belt
    // just starts to slip, then back off.
    private static final double RAMP_START_RPM         = 300;
    private static final double RAMP_RPM_PER_S         = 2000;  // back from 3000 (2026-11-17); at 2000 it kept up (lead 450-620), 4435 in 2.2 s
    private static final double RAMP_MAX_LEAD_RPM      = 800;   // was 400: spin-up took ~4 s

    // == Hood ================================================================
    // Hood angle is in degrees from vertical, forward positive.  Two measured points define the
    // (linear) angle -> servo position conversion; the hood is limited to the range between them.
    // hoodAngleToServo() is the only place that knows about servo positions.
    private static final double HOOD_ANGLE_A_DEG = 0;
    private static final double HOOD_SERVO_A     = 0.64;
    private static final double HOOD_ANGLE_B_DEG = 25;
    private static final double HOOD_SERVO_B     = 0.96;
    public  static final double HOOD_MIN_DEG     = Math.min(HOOD_ANGLE_A_DEG, HOOD_ANGLE_B_DEG);
    public  static final double HOOD_MAX_DEG     = Math.max(HOOD_ANGLE_A_DEG, HOOD_ANGLE_B_DEG);

    /** Servo position for a hood angle (degrees), clipped to the calibrated range. */
    static double hoodAngleToServo(double deg) {
        deg = Range.clip(deg, HOOD_MIN_DEG, HOOD_MAX_DEG);
        double t = (deg - HOOD_ANGLE_A_DEG) / (HOOD_ANGLE_B_DEG - HOOD_ANGLE_A_DEG);
        return HOOD_SERVO_A + t * (HOOD_SERVO_B - HOOD_SERVO_A);
    }

    // == Shot table ==========================================================
    // TODO placeholder: measure with util/ShooterCalibrationOpMode, then run tools/shotcal.py,
    // which rewrites everything between the BEGIN/END markers below (don't edit by hand there).
    // Rows sorted by distance: { distance (in), flywheel RPM, hood angle (deg), flight time (s) }
    // BEGIN GENERATED SHOT_TABLE
    private static final double[][] SHOT_TABLE = {
        {   14.6,  4000,   3.0, 0.732 },
        {   29.4,  4450,  18.0, 0.708 },
        {   39.1,  4500,  21.0, 0.693 },
        {   45.5,  4900,  25.0, 0.682 },
        {   48.4,  4950,  25.0, 0.677 },
        {   60.8,  4900,  25.0, 0.658 },
        {   69.4,  4700,  25.0, 0.643 },
        {   84.2,  5100,  25.0, 0.620 },
    };
    // END GENERATED SHOT_TABLE

    // Number of iterations used to converge the shoot-on-the-move solution.
    private static final int LEAD_ITERATIONS = 3;

    public enum Mode { IDLE, MANUAL, TRACKING }

    private final OpMode   opMode;
    private final Follower drive;
    private DcMotorEx turret;
    private DcMotorEx flywheel;
    private Servo     hood;

    private Mode    mode    = Mode.IDLE;
    private boolean requireChassisAim = false;   // see setRequireChassisAim()
    private BooleanSupplier shooting = () -> false;   // see setShootingSupplier()
    private BooleanSupplier boostAllowed = () -> false;   // see setBoostSupplier()
    private boolean boosting         = false;    // flywheel at BOOST_POWER to recover speed
    private VoltageSensor battery;
    private double  volts            = NOMINAL_VOLTS;   // battery, read every VOLTS_PERIOD_S
    private final ElapsedTime voltsTimer = new ElapsedTime();
    private final ElapsedTime loopTimer  = new ElapsedTime();
    private double  ffPower          = 0;        // this loop's feedforward part of the power
    private double  integralPower    = 0;        // integral part, kept between loops
    private double  flywheelPower    = 0;        // power sent this loop
    private double  lastSentPower    = Double.NaN;
    private boolean ramping          = false;    // spinning up from a stop at RAMP_RPM_PER_S
    private double  commandRPM       = 0;        // speed asked of the closed loop (targetRPM unless ramping)
    private final ElapsedTime rampTimer = new ElapsedTime();
    private boolean redAlliance = true;
    private boolean farGoal     = true;     // which of the alliance's two goals is targeted
    private double  dFar        = 0;        // robot distance to each alliance goal (inches)
    private double  dAudience   = 0;

    // Setpoints
    private double targetRPM        = 0;
    private double targetHoodDeg    = HOOD_MIN_DEG;
    private double targetTurretRad  = 0;
    private double actualRPM        = 0;
    private double actualTurretRad  = 0;

    // Latest solution (also computed when not tracking, for telemetry and tuning)
    private double distance         = 0;
    private Shot   shot             = new Shot(0, 0, 0, HOOD_MIN_DEG);
    private double turretX          = 0;    // turret pivot, field coordinates
    private double turretY          = 0;

    // == standard Methods ===================================
    public ShooterSubsystem(OpMode myOpMode, Follower drive) {
        opMode   = myOpMode;
        this.drive = drive;
    }

    public void init() {
        targetHoodDeg = HOOD_MIN_DEG;

        // All three are always commanded; the *_SIMULATED flags only change what is read back.
        turret   = opMode.hardwareMap.get(DcMotorEx.class, "turret");
        flywheel = opMode.hardwareMap.get(DcMotorEx.class, "shooter");
        hood     = opMode.hardwareMap.get(Servo.class, "hood");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Our own speed loop (see FLYWHEEL_KV): plain power; the encoder is still read for speed.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        battery = opMode.hardwareMap.voltageSensor.iterator().hasNext()
                ? opMode.hardwareMap.voltageSensor.iterator().next() : null;
        readVolts();

        // Move the hood to its minimum angle now, rather than waiting for the first periodic() after start.
        hood.setPosition(hoodAngleToServo(targetHoodDeg));

        idle();
    }

    @Override
    public void periodic() {

        // One read of each per loop.  Simulated: read back what was last commanded instead.
        actualRPM       = SHOOTER_SIMULATED ? commandRPM : ticksPerSecToRpm(flywheel.getVelocity());
        actualTurretRad = TURRET_SIMULATED  ? targetTurretRad : turret.getCurrentPosition() / TURRET_TICKS_PER_RAD;
        if (voltsTimer.seconds() >= VOLTS_PERIOD_S) readVolts();

        computeSolution();

        if (mode == Mode.TRACKING) {
            targetRPM       = shot.rpm;
            targetHoodDeg   = shot.hoodDeg;
            targetTurretRad = TURRET_ENABLED ? turretAngleTo(shot.aimX, shot.aimY) : 0;
        }

        updateRamp();
        updateBoost();
        applyOutputs();
        showTelemetry();
    }

    void showTelemetry() {
        if (SHOOTER_SIMULATED || TURRET_SIMULATED) {
            opMode.telemetry.addLine("SIMULATED: " + (SHOOTER_SIMULATED ? "flywheel speed " : "")
                    + (TURRET_SIMULATED ? "turret angle " : "") + "(read back as commanded)");
        }
        // Shooter  TRACKING RED FAR  45.2 in  READY
        opMode.telemetry.addData("Shooter", "%s %s %s  %4.1f in  %s",
                mode, redAlliance ? "RED" : "BLUE", farGoal ? "FAR" : "AUD", distance,
                mode == Mode.IDLE ? "" : isReady() ? "READY" : !isAtSpeed() ? "spinning up" : "aiming");
        // BLOCKED: only while a shot is wanted (fire held) and the shooter won't take a ball yet:
        // every failing check with its numbers (see blockedReason()).
        if (shooting.getAsBoolean() && !isReady()) {
            opMode.telemetry.addData("BLOCKED", blockedReason());
        }
        // Flywheel  2600 T/A 2580 rpm  hood 14.0  (T/A = target / actual; how the speed is held)
        opMode.telemetry.addData("Flywheel", "%4.0f T/A %4.0f rpm  pwr %.2f  hood %4.1f  %s",
                targetRPM, getActualRPM(), flywheelPower, targetHoodDeg,
                ramping ? String.format(Locale.US, "RAMP %.0f", commandRPM) : boosting ? "BOOST" : "");
        // Turret target / actual.  Aim: where the turret should point to hit the cell, computed
        // every loop even when not tracking: relative to robot forward (+ CCW, before soft limits).
        opMode.telemetry.addData("Turret", "%4.1f T/A %4.1f  aim %4.1f  %s",
                Math.toDegrees(targetTurretRad), Math.toDegrees(getactualTurretRad()),
                Math.toDegrees(getAimError()), TURRET_ENABLED ? "turret aims" : "CHASSIS aims");
        // Distance to each alliance goal, and the Pedro field direction to the aim point.
        opMode.telemetry.addData("Goals", "aud %4.1f  far %4.1f  field hdg %4.0f",
                dAudience, dFar, Math.toDegrees(getAimHeading()));
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
    public void setManual(double rpm, double hoodDeg, double turretRad) {
        mode            = Mode.MANUAL;
        targetRPM       = rpm;
        targetHoodDeg   = hoodDeg;
        targetTurretRad = turretRad;
    }

    /**
     * With TURRET_ENABLED false (the chassis aims), require the chassis to point at the goal
     * (within AIM_TOLERANCE_RAD) before isReady() while tracking.  Teleop turns this on (it turns
     * the chassis to aim); auto leaves it off, since its paths don't turn to face the goal, so auto
     * shoots wherever the path points the robot.  Ignored while TURRET_ENABLED (the turret aims).
     */
    public void setRequireChassisAim(boolean require) {
        requireChassisAim = require;
    }

    /** True while a shot is wanted (normally IntakeSubsystem::isShooting); shows the BLOCKED line. */
    public void setShootingSupplier(BooleanSupplier shooting) {
        this.shooting = shooting;
    }

    /**
     * True while the full-power speed recovery (boost) may run: in BiobuzzRobot, shoot held or the
     * intake full, so the flywheel is already at speed when the driver fires.
     */
    public void setBoostSupplier(BooleanSupplier boostAllowed) {
        this.boostAllowed = boostAllowed;
    }

    /** Flywheel is at full open-loop power to recover its speed. */
    public boolean isBoosting() {
        return boosting;
    }

    /** Chassis points at the aim point, within AIM_TOLERANCE_RAD. */
    public boolean isAimed() {
        return Math.abs(getAimError()) < AIM_TOLERANCE_RAD;
    }

    /** True when the turret is on target: within TURRET_TOLERANCE_RAD and able to reach the aim point. */
    private boolean isTurretOnTarget() {
        return Math.abs(targetTurretRad - getactualTurretRad()) < TURRET_TOLERANCE_RAD && turretTargetReachable();
    }

    /** The chassis heading has to be checked: the chassis aims (no turret) and teleop asked for it. */
    private boolean chassisAimRequired() {
        return !TURRET_ENABLED && requireChassisAim && mode == Mode.TRACKING;
    }

    /**
     * True when the flywheel is at speed and the shooter points at the goal: the turret on target
     * (TURRET_ENABLED), or the chassis aimed (turret disabled, if setRequireChassisAim).
     */
    public boolean isReady() {
        boolean onTarget = TURRET_ENABLED ? isTurretOnTarget()
                                          : !chassisAimRequired() || isAimed();
        return mode != Mode.IDLE
                && targetRPM > 0
                && isAtSpeed()
                && onTarget;
    }

    /**
     * Why isReady() is false, with the numbers: every check that is failing now, e.g.
     * "speed 3650/4125 (-475, tol 400)  chassis aim 12.3 deg (tol 10)".  Null when ready.
     * Mirrors isReady() check by check; keep the two in step.
     */
    public String blockedReason() {
        if (isReady()) return null;
        StringBuilder sb = new StringBuilder();
        if (mode == Mode.IDLE) sb.append("shooter IDLE  ");
        if (targetRPM <= 0) sb.append("no target rpm  ");
        if (!isAtSpeed()) {
            double err = getActualRPM() - targetRPM;
            sb.append(String.format(Locale.US, "speed %.0f/%.0f (%+.0f, tol %.0f)%s  ",
                    getActualRPM(), targetRPM, err, RPM_TOLERANCE, ramping ? " RAMP" : ""));
        }
        if (TURRET_ENABLED) {
            double turretErr = Math.toDegrees(targetTurretRad - getactualTurretRad());
            if (Math.abs(turretErr) >= Math.toDegrees(TURRET_TOLERANCE_RAD)) {
                sb.append(String.format(Locale.US, "turret %+.1f deg (tol %.0f)  ",
                        turretErr, Math.toDegrees(TURRET_TOLERANCE_RAD)));
            }
            if (!turretTargetReachable()) {
                sb.append(String.format(Locale.US, "turret can't reach %.0f deg (limit %.0f..%.0f)  ",
                        Math.toDegrees(getAimError()), Math.toDegrees(TURRET_MIN_RAD), Math.toDegrees(TURRET_MAX_RAD)));
            }
        }
        if (chassisAimRequired() && !isAimed()) {
            sb.append(String.format(Locale.US, "chassis aim %+.1f deg (tol %.0f)  ",
                    Math.toDegrees(getAimError()), Math.toDegrees(AIM_TOLERANCE_RAD)));
        }
        return sb.length() == 0 ? "unknown" : sb.toString().trim();
    }

    /** Flywheel within RPM_TOLERANCE of its target. */
    public boolean isAtSpeed() {
        return Math.abs(getActualRPM() - targetRPM) < RPM_TOLERANCE;
    }

    /** Pedro field direction (radians) from the turret to the lead-corrected aim point. */
    public double getAimHeading() {
        return Math.atan2(shot.aimY - turretY, shot.aimX - turretX);
    }

    /** How far to turn the robot (radians, + is CCW) to point the shooter (turret at 0) at the aim point. */
    public double getAimError() {
        return Angle.normalizeSigned(getAimHeading() - drive.pose().heading() - TURRET_ZERO_RAD);
    }

    /** Distance (inches) from the turret to the (virtual) target. */
    public double getDistance() {
        return distance;
    }

    public Mode getMode() {
        return mode;
    }

    /** Flywheel speed as read this loop (the commanded speed while SHOOTER_SIMULATED). */
    public double getActualRPM() {
        return actualRPM;
    }

    /** Same as getActualRPM() (kept for the calibration opmode). */
    public double getMeasuredRPM() {
        return actualRPM;
    }

    /** The goal being aimed at: the alliance goal nearer the robot (always on the robot's side). */
    public Pose getGoal() {
        return target();
    }

    /** Turret angle as read at the start of this loop's periodic() (one encoder read per loop). */
    public double getactualTurretRad() {
        return actualTurretRad;     // the commanded angle while TURRET_SIMULATED
    }

    public double getTargetRPM()       { return targetRPM; }
    /** Speed asked of the motor this loop: the target, or the spin-up ramp's value while ramping. */
    public double getCommandRPM()      { return commandRPM; }
    /** Flywheel motor power sent this loop, its feedforward part, and its integral part. */
    public double getFlywheelPower()   { return flywheelPower; }
    public double getFeedforwardPower() { return ffPower; }
    public double getIntegralPower()   { return integralPower; }
    /** Battery volts, read every VOLTS_PERIOD_S (used for the flywheel's voltage compensation). */
    public double getBatteryVolts()    { return volts; }
    public double getTargetHoodDeg()   { return targetHoodDeg; }
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

    /** Turret angle (from its zero, which faces TURRET_ZERO_RAD from robot forward) that points at a field location. */
    private double turretAngleTo(double x, double y) {
        double fieldAngle = Math.atan2(y - turretY, x - turretX);
        double angle = Angle.normalizeSigned(fieldAngle - drive.pose().heading() - TURRET_ZERO_RAD);
        return Range.clip(angle, TURRET_MIN_RAD, TURRET_MAX_RAD);
    }

    /** False if the aim point is outside the turret's travel (clipped at a soft limit). */
    private boolean turretTargetReachable() {
        if (mode != Mode.TRACKING) return true;
        double fieldAngle = Math.atan2(shot.aimY - turretY, shot.aimX - turretX);
        double angle = Angle.normalizeSigned(fieldAngle - drive.pose().heading() - TURRET_ZERO_RAD);
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

    /**
     * Spin-up ramp.  A start from (nearly) stopped begins a ramp from the measured speed; while it
     * runs, the speed asked for rises at RAMP_RPM_PER_S (never above the target).  It ends when it
     * reaches the target, or when the flywheel is told to stop.
     */
    private void updateRamp() {
        double dt = Math.min(rampTimer.seconds(), 0.1);
        rampTimer.reset();

        if (targetRPM <= 0) {
            ramping = false;
        } else if (!ramping && actualRPM < RAMP_START_RPM && targetRPM > RAMP_START_RPM) {
            ramping = true;
            commandRPM = Math.max(actualRPM, 0);
        }

        if (ramping) {
            // Rise at RAMP_RPM_PER_S, but never run far ahead of the flywheel itself.
            commandRPM = Math.min(targetRPM, Math.min(commandRPM + RAMP_RPM_PER_S * dt,
                                                      actualRPM + RAMP_MAX_LEAD_RPM));
            if (commandRPM >= targetRPM) ramping = false;
        } else {
            commandRPM = Math.max(targetRPM, 0);
        }
    }

    /** True while the flywheel is ramping up from a stop. */
    public boolean isRamping() {
        return ramping;
    }

    /**
     * Boost (full open-loop power) while allowed (setBoostSupplier) whenever the flywheel is more than
     * BOOST_ENTER_RPM below its target; stop once it is within BOOST_EXIT_RPM (or when no longer allowed).
     */
    private void updateBoost() {
        double slow = targetRPM - actualRPM;                // positive: too slow
        if (!BOOST_ENABLED || targetRPM <= 0 || ramping || !boostAllowed.getAsBoolean()) {
            boosting = false;
        } else if (!boosting) {
            boosting = slow > BOOST_ENTER_RPM;
        } else {
            boosting = slow > BOOST_EXIT_RPM;
        }
    }

    private void readVolts() {
        voltsTimer.reset();
        if (battery != null) {
            double v = battery.getVoltage();
            if (v > 0) volts = v;
        }
    }

    /**
     * This loop's flywheel power (see FLYWHEEL_KV): voltage-compensated feedforward on commandRPM,
     * plus P and a clamped integral on the error; BOOST_POWER while boosting.
     */
    private double flywheelPower() {
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();
        if (targetRPM <= 0) {
            integralPower = 0;
            ffPower = 0;
            return FLYWHEEL_IDLE_POWER;
        }
        double error = commandRPM - actualRPM;
        ffPower = (FLYWHEEL_KS + FLYWHEEL_KV * commandRPM) / Math.max(volts, MIN_VOLTS);   // volts needed / volts available
        if (boosting) return BOOST_POWER;
        if (!ramping && Math.abs(error) < FLYWHEEL_I_ZONE_RPM) {
            integralPower = Range.clip(integralPower + FLYWHEEL_KI * error * dt,
                                       -FLYWHEEL_I_MAX_POWER, FLYWHEEL_I_MAX_POWER);
        }
        return Range.clip(ffPower + FLYWHEEL_KP * error + integralPower, 0, 1);
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

        // Control Flywheel: our own loop holds commandRPM (targetRPM, or the spin-up ramp).  Each
        // setPower is a hub write, so skip changes too small to matter.
        flywheelPower = flywheelPower();
        if (Double.isNaN(lastSentPower) || Math.abs(flywheelPower - lastSentPower) >= POWER_MIN_STEP
                || (flywheelPower == 0) != (lastSentPower == 0)) {
            flywheel.setPower(flywheelPower);
            lastSentPower = flywheelPower;
        }

        // Control Hood Angle: skip changes too small for the servo to see (saves a hub write).
        double hoodPos = hoodAngleToServo(targetHoodDeg);
        if (Math.abs(hoodPos - lastHoodPos) >= HOOD_MIN_STEP) {
            hood.setPosition(hoodPos);
            lastHoodPos = hoodPos;
        }
    }

    // Last values sent to the hub, so unchanged ones aren't sent again.
    private static final double HOOD_MIN_STEP = 0.001;   // servo position; ~0.08 deg of hood
    private double lastHoodPos       = -1;   // never a real position, so the first one is sent

    private static class Shot {
        final double aimX, aimY, rpm, hoodDeg;
        Shot(double aimX, double aimY, double rpm, double hoodDeg) {
            this.aimX = aimX;
            this.aimY = aimY;
            this.rpm  = rpm;
            this.hoodDeg = hoodDeg;
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

    public Command manualCommand(double rpm, double hoodDeg, double turretRad) {
        return new InstantCommand(() -> setManual(rpm, hoodDeg, turretRad), this);
    }

    /** Finishes when ready to shoot, or after timeoutMs. Use before the transfer's feed command. */
    public Command waitUntilReadyCommand(long timeoutMs) {
        return new WaitUntilCommand(this::isReady).withTimeout(timeoutMs);
    }
}
