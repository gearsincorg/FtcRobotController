package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.FunctionalCommand;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.AutoConfig;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

public class BiobuzzRobot {
    public IntakeSubsystem intake;
    public VisionSubsystem vision;
    public ShooterSubsystem shooter;
    public Follower drive;
    public AutoConfig config;

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

        // While a shot is wanted the shooter recovers its speed at full power after each ball.
        shooter.setShootingSupplier(intake::isShooting);

        // Start on the alliance chosen in the auto setup menu (Share / Options still override).
        config = new AutoConfig(opMode);
        config.initialize();
    }

    /***
     * Update any required robot functions.
     */
    public void update() {
        drive.update();
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
     * Turn power (+ is counter-clockwise, like Pedro) that removes a heading error, limited to
     * +/- maxTurn.  errorRad is target minus actual.  This mirrors Foresight.headingFeedback()
     * (which is private): the tuned headingFeedback controller plus headingStaticFF in the
     * direction of the error, so manual driving gets the same friction boost as paths.  No
     * deadband, exactly as Foresight; add one here AND for paths only if buzzing at the target
     * proves it necessary.
     */
    public double turnForError(double errorRad, double maxTurn) {
        double error = Angle.normalizeSigned(errorRad);
        double turn  = headingController().calculate(0, error)
                     + headingStaticFF().calculate(0, Angle.turnDirection(error));
        return Range.clip(turn, -maxTurn, maxTurn);
    }

    /** Turn power to point the robot at a field heading (radians), limited to +/- maxTurn. */
    public double turnToHeading(double targetRad, double maxTurn) {
        return turnForError(targetRad - drive.pose().heading(), maxTurn);
    }

    /**
     * Auto step mode: with step mode on (auto setup menu), wait here until gamepad 1 Cross is pressed;
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
