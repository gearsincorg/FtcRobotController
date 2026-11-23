package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.AutoConfig;
import org.firstinspires.ftc.teamcode.util.ShooterLog;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

public class BiobuzzRobot {
    public IntakeSubsystem intake;
    public VisionSubsystem vision;
    public ShooterSubsystem shooter;
    public Follower drive;
    public AutoConfig config;

    // Per-loop shooter log (util/ShooterLog): one CSV row per loop for the whole run, on the hub in
    // /sdcard/FIRST/shooter_log/.  Pull and plot with `python tools/shotlog.py`.  false = off.
    public static final boolean SHOOTER_LOG_ENABLED = true;
    public ShooterLog shooterLog;

    private OpMode  opMode;

    public static final ColorRange POLLEN_YELLOW = new ColorRange(
        ColorSpace.YCrCb,
        new Scalar( 128, 135, 29),
        new Scalar(255, 166, 120)
    );

    public BiobuzzRobot(OpMode opmode) {
        this.opMode = opmode;
    }

    public void initialize() {
        initialize(0);
    }

    /** maxPathSpeed caps path following (inches/sec); 0 = no cap. */
    public void initialize(double maxPathSpeed) {
        // Bulk reads: each hub's encoders, velocities and digital inputs come back in ONE
        // transaction per loop instead of one per read (each read costs ~2 ms).  MANUAL: the
        // scheduler clears the cache at the end of every CommandScheduler.run(), so the next loop
        // reads fresh values.  Anything that reads hardware outside the scheduler (e.g. during
        // init_loop) must call clearBulkCache() first, or it sees the same values forever.
        CommandScheduler.getInstance().setBulkReading(opMode.hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        vision = new VisionSubsystem(opMode);
        vision.init();

        // Setup robot
        drive = maxPathSpeed > 0
                ? Constants.create(opMode.hardwareMap, maxPathSpeed)
                : Constants.create(opMode.hardwareMap);

        // Shooter aims from odometry, so it needs the drive's Follower.
        shooter = new ShooterSubsystem(opMode, drive);
        shooter.init();

        // Intake waits for the shooter to be ready before feeding each ball.
        intake = new IntakeSubsystem(opMode, shooter::isReady);
        intake.init();

        // Shoot held shows why a shot is blocked.  Boost (full-power speed recovery) runs while shoot
        // is held (after each ball) or the intake is full (ready before the driver fires).
        shooter.setShootingSupplier(intake::isShooting);
        shooter.setBoostSupplier(() -> intake.isShooting() || intake.isFull());

        // Start on the alliance chosen in the auto setup menu (Share / Options still override).
        config = new AutoConfig(opMode);
        config.initialize();

        if (SHOOTER_LOG_ENABLED) {
            shooterLog = new ShooterLog(opMode);
            shooterLog.start(opMode.getClass().getSimpleName());
        }
    }

    /***
     * Update any required robot functions.
     */
    public void update() {
        drive.update();
        if (startPoseSet) savedPose = drive.pose();   // for the next opmode (see savedPose)
        if (shooterLog != null) shooterLog.row(shooter, intake);   // after the subsystems ran this loop
    }

    /** One telemetry line about the shooter log: its file and row count, or why it isn't logging. */
    public String shooterLogStatus() {
        return shooterLog == null ? "off" : shooterLog.getStatus();
    }

    // == Odometry start ========================================================

    /**
     * The odometry does NOT carry over between opmodes: Pedro's OctoQuadLocalizer constructor calls
     * resetLocalizerAndCalibrateIMU(), so every opmode starts at (0, 0, 0).  So the pose is saved
     * here, every loop (update()), by any opmode that was given a known start pose (auto, teleop,
     * Shooter Calibration), and teleop restores it at init with restoreSavedPose().  Static, so it
     * survives opmode changes; null again after the Robot Controller app restarts (a power cycle
     * or a new deploy).  An opmode that never set a start pose (e.g. a test opmode starting at
     * (0, 0, 0)) doesn't overwrite it.
     */
    private static Pose savedPose = null;
    private boolean startPoseSet = false;   // this opmode has set a start pose, so its pose is real

    /** The pose the last opmode with a known start ended at, or null if none since the app started. */
    public static Pose getSavedPose() { return savedPose; }

    /** Set the odometry to a known pose and save it.  Use this, not drive.setPose(). */
    public void setStartPose(Pose pose) {
        drive.setPose(pose);
        savedPose = pose;
        startPoseSet = true;
    }

    /** Set the odometry to the saved pose (where the last opmode ended).  False if there is none. */
    public boolean restoreSavedPose() {
        if (savedPose == null) return false;
        setStartPose(savedPose);
        return true;
    }

    // == Current path (for telemetry) ==========================================

    private String   pathName  = null;
    private String[] lineNames = new String[0];

    /** Called by NamedFollowPathCommand when a generated path starts. */
    public void setCurrentPath(String name, String[] lines) {
        pathName  = name;
        lineNames = lines;
    }

    /**
     * The path being followed (or the last one, while holding its end), and the line within it:
     * e.g. "NearCenter_2 > point2 (2/2)".  "none" before the first path.
     */
    public String pathStatus() {
        if (pathName == null) return "none";
        int i = drive.pathIndex();
        if (i < 0 || i >= lineNames.length) return pathName;
        return String.format("%s > %s (%d/%d)", pathName, lineNames[i], i + 1, lineNames.length);
    }

    /** Throw away the bulk-read cache so the next hardware read is fresh (for reads outside the scheduler). */
    public void clearBulkCache() {
        for (LynxModule hub : opMode.hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }
    }

    // == Heading control for manual driving ====================================

    private static final Controller FALLBACK_HEADING = Controller.proportional(2.0);

    /** The heading controller Foresight uses for path following (tuned in pedro/Constants). */
    public Controller headingController() {
        return drive.algorithm() instanceof Foresight
                ? ((Foresight) drive.algorithm()).config.headingFeedback.get()
                : FALLBACK_HEADING;
    }

    /** Clear the controller's memory; call when a heading lock or turn starts. */
    public void resetHeadingController() {
        headingController().reset();
    }

    private static final Controller NO_FEEDFORWARD = Controller.zero;

    /** Foresight's static heading feedforward (pedro/Constants), or none if it isn't Foresight. */
    private Controller headingStaticFF() {
        return drive.algorithm() instanceof Foresight
                ? ((Foresight) drive.algorithm()).config.headingStaticFF.get()
                : NO_FEEDFORWARD;
    }

    /**
     * How far (radians, + is CCW) the robot would keep turning if it braked now, from its turn rate
     * and Foresight's headingBrakeCoefficients (x = linear, y = quadratic):
     * omega * linear + omega * |omega| * quadratic.  Same model Foresight uses (getBrakeDisplacement).
     */
    public double headingBrakeDisplacement() {
        if (!(drive.algorithm() instanceof Foresight)) return 0;
        Vector2D k = ((Foresight) drive.algorithm()).config.headingBrakeCoefficients.get();
        double omega = drive.velocity().omega;
        return omega * k.x() + omega * Math.abs(omega) * k.y();
    }

    /**
     * Turn power (+ is counter-clockwise, like Pedro) that removes a heading error, limited to
     * +/- maxTurn.  errorRad is target minus actual.  This mirrors what Foresight does when it
     * follows a path: the error is measured from where the heading would stop if the robot braked
     * now (heading + headingBrakeDisplacement()), not from where it is.  That look-ahead is the
     * loop's damping: without it the P gain (about 2.6 per radian) overshoots and oscillates after
     * a large error (a bump, or turning while driving).  Driving and paths use the same
     * coefficients, so they feel the same; tune the damping there (pedro/Constants).  Then the tuned headingFeedback controller
     * plus headingStaticFF in the direction of the error, as Foresight.headingFeedback() (private).
     * No deadband, exactly as Foresight.
     */
    public double turnForError(double errorRad, double maxTurn) {
        double error = Angle.normalizeSigned(errorRad - headingBrakeDisplacement());
        double turn  = headingController().calculate(0, error)
                     + headingStaticFF().calculate(0, Angle.turnDirection(error));
        return Range.clip(turn, -maxTurn, maxTurn);
    }

    /** Turn power to point the robot at a field heading (radians), limited to +/- maxTurn. */
    public double turnToHeading(double targetRad, double maxTurn) {
        return turnForError(targetRad - drive.pose().heading(), maxTurn);
    }

    // == Auto aiming (turret disabled: the chassis aims) ========================

    /** Give up turning to the goal after this long and shoot anyway (the volley still needs isAimed()). */
    public static final long   AIM_TIMEOUT_MS        = 2000;
    /** Aimed counts only once the turn has (nearly) stopped, so the volley doesn't start mid-swing. */
    private static final double AIM_SETTLED_TURN_RATE = Math.toRadians(20);   // rad/s
    /**
     * Longest an auto volley may take from when shoot is asked for, spin-up and aiming included.
     * shootAllCommand's own VOLLEY_TIMEOUT_MS only starts at the first feed, so if the shooter never
     * gets ready (flywheel short of target, chassis never aimed) the volley would wait forever.
     */
    public static final long   VOLLEY_MAX_MS         = 3500;
    /** Re-send the hold heading when the aim moves this much (the turret pivot moves as the robot turns). */
    private static final double AIM_RETARGET_RAD      = Math.toRadians(1);

    /**
     * Auto: turn in place until the shooter points at the goal, using Pedro's hold (so the turn uses
     * the same Foresight heading controller as paths; no separate gains).  Holds the x/y where it
     * started and the heading pose.heading + shooter.getAimError(), re-aimed as that moves.  Finishes
     * when shooter.isAimed() and the turn has settled; at once while TURRET_ENABLED (the turret
     * aims).  Pedro keeps holding that pose afterwards, until the next path.  Use with a timeout
     * (aimAndShootCommand does).
     */
    public Command aimAtGoalCommand() {
        double[] hold = new double[3];   // x, y, heading being held
        return new FunctionalCommand(
                () -> {
                    if (ShooterSubsystem.TURRET_ENABLED) return;
                    Pose p = drive.pose();
                    hold[0] = p.x();
                    hold[1] = p.y();
                    hold[2] = p.heading() + shooter.getAimError();
                    drive.hold(new Pose(hold[0], hold[1], hold[2]));
                },
                () -> {
                    if (ShooterSubsystem.TURRET_ENABLED) return;
                    double heading = drive.pose().heading() + shooter.getAimError();
                    if (Math.abs(Angle.normalizeSigned(heading - hold[2])) >= AIM_RETARGET_RAD) {
                        hold[2] = heading;
                        drive.hold(new Pose(hold[0], hold[1], hold[2]));
                    }
                },
                interrupted -> {},
                () -> ShooterSubsystem.TURRET_ENABLED
                        || (shooter.isAimed() && Math.abs(drive.velocity().omega) < AIM_SETTLED_TURN_RATE));
    }

    /**
     * Auto volley: turn to the goal (at most AIM_TIMEOUT_MS), then fire every ball
     * (intake.shootAllCommand(), at most VOLLEY_MAX_MS; on timeout shoot is released and any balls
     * left stay in the column).  The shooter must already be tracking.  Auto turns on
     * setRequireChassisAim(true), so no ball is fed while the chassis is off target.
     */
    public Command aimAndShootCommand() {
        return new SequentialCommandGroup(
                aimAtGoalCommand().withTimeout(AIM_TIMEOUT_MS),
                intake.shootAllCommand().withTimeout(VOLLEY_MAX_MS));
    }

    /**
     * Auto step mode:with step mode on (auto setup menu), wait here until gamepad 1 Cross is pressed;
     * with it off, finish at once.  Cross must be released and pressed again, so holding it doesn't
     * skip several pauses.  Pedro holds the robot at the end of the last path while paused.
     */
    public Command stepCommand(String label) {
        boolean[] released = { false };
        return new FunctionalCommand(
                () -> released[0] = false,
                () -> {
                    if (config.stepMode) {
                        opMode.telemetry.addLine("PAUSED " + label + ": press Cross to continue");
                    }
                },
                interrupted -> {},
                () -> {
                    if (!config.stepMode) return true;
                    if (!opMode.gamepad1.cross) {
                        released[0] = true;
                        return false;
                    }
                    return released[0];
                });
    }
}
