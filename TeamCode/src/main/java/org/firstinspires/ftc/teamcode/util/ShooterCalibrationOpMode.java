package org.firstinspires.ftc.teamcode.util;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.BiobuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Shooter calibration: collects the data for ShooterSubsystem.SHOT_TABLE.
 *
 * Shoot from anywhere: drive there by hand, or pick a preset point (a distance and an angle off the
 * front of the goal on the robot's side) and drive to it with X.  Load ONE ball and shoot it (RB) at
 * the RPM and hood you set.  The shot is saved where it was taken (pose and distance to the goal),
 * whether or not that is a preset point.
 * Press LB when it enters the goal to time its flight.  Repeat, adjusting RPM and hood, until the
 * shots are consistent, then save.  What is saved is the LAST SHOT (its RPM and hood as they were
 * when it left, the flywheel speed measured then, battery volts, flight time, field position), not
 * the current settings, so you can nudge things after the shot without spoiling the data.
 *
 *   Share      saves it as a best shot (feeds SHOT_TABLE)
 *   Left stick press saves it as a COMPENSATION shot: an intentionally different RPM (normally lower)
 *              with the hood that works for it.  Not used for SHOT_TABLE; only for working out hood compensation.
 * Every save adds a row: nothing is replaced, and repeats at the same spot are all kept.
 *
 * Rows are written to /sdcard/FIRST/shooter_cal.csv on the hub.  On the PC, `python tools/shotcal.py`
 * pulls that file and rewrites SHOT_TABLE in ShooterSubsystem.java.
 *
 * Start with the robot at CAL_START_POSE and the turret centered.
 *
 *   Sticks              : drive (field centric, half speed); cancels a drive-to
 *   Right stick press   : aim at the goal on / off (starts off; twisting the right stick overrides)
 *   D-pad up / down     : select distance;  left / right : select angle off the goal's front
 *   X                   : drive to the selected point, then aim at the goal
 *   B                   : flywheel on / off
 *   Y / A               : RPM +/- 100
 *   Right / left trigger: hood +/- 1 degree
 *   Right bumper (hold) : shoot
 *   Left bumper         : the ball went in (flight time of the last shot)
 *   Share               : save the last shot as a best shot
 *   Left stick press    : save the last shot as a compensation shot
 *   Options (twice)     : undo this session's last save
 *   Touchpad            : reset pose to CAL_START_POSE
 */
@TeleOp(name = "Shooter Calibration", group = "Tuning")
public class ShooterCalibrationOpMode extends CommandOpMode {

    // == Setup ===============================================================
    // TODO placeholder: a known, easy-to-reproduce starting spot (the Auto1 start position).
    private static final Pose CAL_START_POSE = new Pose(32.75, 6, Math.toRadians(180));

    // Calibration points: distance from the goal (inches) x angle off the goal's front (degrees,
    // + is CCW seen from above).  0 is straight in front of the goal.
    private static final double   MIN_DIST   = 12;
    private static final double   MAX_DIST   = 96;
    private static final double   DIST_STEP  = 3;
    private static final double[] DISTANCES  = steps(MIN_DIST, MAX_DIST, DIST_STEP);     // 12, 15, 18, ... 96
    private static final double   MAX_ANGLE  = 60;
    private static final double   ANGLE_STEP = 15;
    private static final double[] ANGLES     = steps(-MAX_ANGLE, MAX_ANGLE, ANGLE_STEP);  // -60, -45, ... 60

    // Pedro field direction (degrees) that each goal's front faces.  TODO measure on the field.
    private static final double AUDIENCE_GOAL_FACING_DEG = 270;
    private static final double FAR_GOAL_FACING_DEG      = 90;

    // Points closer than this to a wall (half a robot) are off the field.
    private static final double FIELD_MARGIN = 9;

    private static final String CSV_NAME = "shooter_cal.csv";
    private static final String BACKUP_TAG = ".bak-";       // backups: shooter_cal.csv.bak-yyyyMMdd-HHmmss
    private static final int    MAX_BACKUPS = 5;
    // Columns: rpm and hood_deg are the settings of the shot; rpm_meas is the measured flywheel speed
    // as the ball left.  kind is "best" or "comp"; old rows (12 columns) have none and count as best.
    private static final String CSV_HEADER =
            "time,goal,set_dist,set_angle,x,y,heading_deg,dist,rpm,hood_deg,tof_s,tof_n,kind,rpm_meas,volts";
    private static final int    COL_KIND = 12;
    private static final String KIND_BEST = "best";
    private static final String KIND_COMP = "comp";

    // Aiming: Foresight's heading controller (robot.turnForError), limited to this turn power
    private static final double MAX_AIM_TURN = 0.7;
    private static final double DRIVE_SCALE  = 0.5;
    // With the sticks idle the robot holds its pose once it is slower than these (in/s, rad/s).
    private static final double HOLD_SPEED_IPS = 1.0;
    private static final double HOLD_TURN_RATE = 0.1;

    // Flight time: an LB press more than this long after a launch doesn't count.
    private static final double MAX_FLIGHT_S = 2.0;
    // How long the measured flywheel speed is remembered, to look up its value at the moment of a launch.
    private static final double RPM_HISTORY_S = 1.5;
    private static final double AT_SPEED_RPM = 100;

    // == State ===============================================================
    private BiobuzzRobot robot;
    private int     distIndex  = (int) Math.round((36 - MIN_DIST) / DIST_STEP);   // start at 36 in
    private int     angleIndex = (int) Math.round(MAX_ANGLE / ANGLE_STEP);         // start at 0 deg
    private double  rpm        = 3000;
    private double  hoodDeg    = 12;
    private boolean flywheelOn = true;
    private boolean aiming     = false;     // chassis points at the goal (right stick press, or after X)

    private FollowPathCommand driveTo;
    private Pose    driveTarget;            // where driveTo is going
    private boolean holding   = false;      // Pedro is holding a pose (sticks idle)
    private boolean holdAimed = false;      // the aiming setting the hold was made with

    /** Everything about the last shot, taken as it left; this is what gets saved. */
    private static class Shot {
        double launchT;                 // clock time of the launch (s)
        double rpmSet, hood;            // the settings it was shot with
        double rpmMeas;                 // measured flywheel speed as it left
        double volts;
        double x, y, headingDeg, dist;  // robot pose and distance to the goal
        double tof = Double.NaN;        // flight time, once LB is pressed
        boolean savedBest, savedComp;   // so one shot isn't saved twice by a double press
    }

    private final ElapsedTime        clock     = new ElapsedTime();
    private final ArrayDeque<double[]> rpmHistory = new ArrayDeque<>();   // { time (s), measured rpm }
    private VoltageSensor battery;
    private Shot    lastShot;                   // null until the first shot
    private int     lastBallCount = 0;
    private boolean lastRT, lastLT;

    // Every row in the file (CSV fields), and the rows saved this session (for undo, newest last).
    private final List<String[]> saved = new ArrayList<>();
    private final List<String>   sessionSaves = new ArrayList<>();
    // Telemetry lists the best shots saved within this distance of the robot's distance to the goal.
    private static final double NEAR_DIST = 3;
    private String  lastSave     = "";
    private boolean deleteArmed  = false;          // Options pressed once; press again to undo
    private final ElapsedTime deleteTimer = new ElapsedTime();
    private static final double DELETE_CONFIRM_S = 2.0;

    @Override
    public void initialize() {
        robot = new BiobuzzRobot(this);
        robot.initialize(25);
        battery = hardwareMap.voltageSensor.iterator().next();
        robot.drive.setPose(CAL_START_POSE);
        robot.shooter.setAlliance(robot.config.red);
        applyShooter();
        loadSaved();

        GamepadEx pad = new GamepadEx(gamepad1);
        pad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(new InstantCommand(() -> selectPoint(1, 0)));
        pad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(new InstantCommand(() -> selectPoint(-1, 0)));
        pad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(new InstantCommand(() -> selectPoint(0, 1)));
        pad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new InstantCommand(() -> selectPoint(0, -1)));
        pad.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(new InstantCommand(this::driveToPoint));
        pad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(new InstantCommand(() -> aiming = !aiming));
        pad.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(new InstantCommand(() -> { flywheelOn = !flywheelOn; applyShooter(); }));
        pad.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(new InstantCommand(() -> adjust(50, 0)));
        pad.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(() -> adjust(-50, 0)));
        pad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whileHeld(robot.intake.shootWhileHeldCommand());
        pad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(new InstantCommand(this::ballIn));
        // On the PS pad: Share = BACK, Options = START
        pad.getGamepadButton(GamepadKeys.Button.BACK)
            .whenPressed(new InstantCommand(() -> saveShot(false)));
        pad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
            .whenPressed(new InstantCommand(() -> saveShot(true)));
        pad.getGamepadButton(GamepadKeys.Button.START)
            .whenPressed(new InstantCommand(this::undoLastSave));
        pad.getGamepadButton(GamepadKeys.Button.TOUCHPAD)
            .whenPressed(new InstantCommand(() -> robot.drive.setPose(CAL_START_POSE)));
    }

    @Override
    public void run() {
        super.run();
        hoodTriggers();
        watchLaunches();
        drive();
        robot.update();
        showTelemetry();
    }

    // == Driving =============================================================

    private void drive() {
        double axial   = -gamepad1.left_stick_y  * DRIVE_SCALE;
        double lateral = -gamepad1.left_stick_x  * DRIVE_SCALE;
        double yaw     = -gamepad1.right_stick_x * DRIVE_SCALE;
        boolean sticks = axial != 0 || lateral != 0 || yaw != 0;

        if (driveTo != null) {
            if (driveTo.isScheduled()) {
                if (!sticks) return;        // Pedro is driving
                driveTo.cancel();           // the driver takes over
            } else {
                aiming = true;              // arrived: aim at the goal
                holdAt(driveTarget);        // and hold the exact point (no drifting past it)
            }
            driveTo = null;
        }

        if (sticks) {
            holding = false;                // the driver has control
        } else {
            if (holding && aiming != holdAimed) holding = false;   // aim toggled: hold again
            if (holding) return;            // Pedro is holding the pose
            if (settled()) {                // sticks released and stopped: hold where we are
                Pose p = robot.drive.pose();
                double heading = aiming ? p.heading() + robot.shooter.getAimError() : p.heading();
                holdAt(new Pose(p.x(), p.y(), heading));
                return;
            }
        }

        if (yaw == 0 && aiming) {
            yaw = robot.turnForError(robot.shooter.getAimError(), MAX_AIM_TURN);
        }
        DrivePowers powers = ManualDrive.fieldCentric(axial, lateral, yaw, robot.drive.pose().heading());
        robot.drive.manual(powers);
    }

    private void driveToPoint() {
        Pose target = selectedPose();
        if (!onField(target)) return;
        if (driveTo != null) driveTo.cancel();
        Pose start = robot.drive.pose();
        Path path = Paths.path(line(start, target).linear(start, target));
        driveTo = new FollowPathCommand(robot.drive, path);
        driveTarget = target;
        holding = false;
        schedule(driveTo);
    }

    /** Have Pedro hold this pose (position and heading) until the driver takes over. */
    private void holdAt(Pose pose) {
        robot.drive.hold(pose);
        holding    = true;
        holdAimed  = aiming;
    }

    /** True once the robot has (nearly) stopped moving and turning. */
    private boolean settled() {
        return Math.hypot(robot.drive.velocity().vx, robot.drive.velocity().vy) < HOLD_SPEED_IPS
            && Math.abs(robot.drive.velocity().omega) < HOLD_TURN_RATE;
    }

    /** Field pose of the selected point, facing the goal. */
    private Pose selectedPose() {
        Pose goal = robot.shooter.getGoal();
        double dir = Math.toRadians(goalFacingDeg(goal) + ANGLES[angleIndex]);
        double d   = DISTANCES[distIndex];
        double x   = goal.x() + d * Math.cos(dir);
        double y   = goal.y() + d * Math.sin(dir);
        return new Pose(x, y, Math.atan2(goal.y() - y, goal.x() - x));
    }

    /** from, from + step, ... to. */
    private static double[] steps(double from, double to, double step) {
        int n = (int) Math.round((to - from) / step) + 1;
        double[] d = new double[n];
        for (int i = 0; i < n; i++) d[i] = from + i * step;
        return d;
    }

    private static double goalFacingDeg(Pose goal) {
        return goal.y() < 72 ? AUDIENCE_GOAL_FACING_DEG : FAR_GOAL_FACING_DEG;
    }

    private static String goalName(Pose goal) {
        return goal.y() < 72 ? "AUDIENCE" : "FAR";
    }

    private static boolean onField(Pose p) {
        return p.x() >= FIELD_MARGIN && p.x() <= 144 - FIELD_MARGIN
            && p.y() >= FIELD_MARGIN && p.y() <= 144 - FIELD_MARGIN;
    }

    private void selectPoint(int dDist, int dAngle) {
        distIndex  = Range.clip(distIndex + dDist, 0, DISTANCES.length - 1);
        angleIndex = Range.clip(angleIndex + dAngle, 0, ANGLES.length - 1);
        deleteArmed = false;
    }

    // == Shooter =============================================================

    private void adjust(double dRpm, double dHoodDeg) {
        rpm     = Range.clip(rpm + dRpm, 0, ShooterSubsystem.FLYWHEEL_MAX_RPM);   // flywheel RPM
        hoodDeg = Range.clip(hoodDeg + dHoodDeg, ShooterSubsystem.HOOD_MIN_DEG, ShooterSubsystem.HOOD_MAX_DEG);
        applyShooter();     // the last shot keeps the settings it was taken with
    }

    private void applyShooter() {
        robot.shooter.setManual(flywheelOn ? rpm : 0, hoodDeg, 0);
    }

    private void hoodTriggers() {
        boolean rt = gamepad1.right_trigger > 0.5, lt = gamepad1.left_trigger > 0.5;
        if (rt && !lastRT) adjust(0, 1);
        if (lt && !lastLT) adjust(0, -1);
        lastRT = rt;
        lastLT = lt;
    }

    // == Flight time =========================================================

    /**
     * A ball left when the count drops while shooting (the count lags by the sensor debounce).
     * Record everything about the shot as it was then; it replaces the previous last shot.
     */
    private void watchLaunches() {
        int count = robot.intake.ballCount();
        double now = clock.seconds();

        rpmHistory.addLast(new double[] { now, robot.shooter.getMeasuredRPM() });
        while (rpmHistory.size() > 1 && now - rpmHistory.peekFirst()[0] > RPM_HISTORY_S) {
            rpmHistory.removeFirst();
        }

        if (count < lastBallCount && robot.intake.getMode() == IntakeSubsystem.Mode.SHOOTING) {
            Pose pose = robot.drive.pose();
            Shot s = new Shot();
            s.launchT    = now - IntakeSubsystem.COUNT_DEBOUNCE_MS / 1000.0;
            s.rpmSet     = rpm;
            s.hood       = hoodDeg;
            s.rpmMeas    = rpmAt(s.launchT);
            s.volts      = battery.getVoltage();
            s.x          = pose.x();
            s.y          = pose.y();
            s.headingDeg = Math.toDegrees(pose.heading());
            s.dist       = robot.shooter.getDistance();
            lastShot = s;
        }
        lastBallCount = count;
    }

    /** Measured flywheel speed at clock time t (nearest remembered sample). */
    private double rpmAt(double t) {
        double best = robot.shooter.getMeasuredRPM(), bestDt = Double.MAX_VALUE;
        for (double[] h : rpmHistory) {
            double dt = Math.abs(h[0] - t);
            if (dt < bestDt) { bestDt = dt; best = h[1]; }
        }
        return best;
    }

    /** LB: the last shot went in; its flight time is now - launch. */
    private void ballIn() {
        if (lastShot == null || !Double.isNaN(lastShot.tof)) return;
        double t = clock.seconds() - lastShot.launchT;
        if (t <= MAX_FLIGHT_S) lastShot.tof = t;
    }

    // == Saving ==============================================================
    // Every save appends one row, from wherever the robot is: nothing is replaced or filtered.
    // Options (twice) undoes this session's last save.  Before every change the old file is copied to
    // shooter_cal.csv.bak-<time> (the newest 5 are kept), so mistakes can be undone.

    private File csvFile() {
        return new File(AppUtil.FIRST_FOLDER, CSV_NAME);
    }

    private static boolean isComp(String[] f) {
        return f.length > COL_KIND && KIND_COMP.equals(f[COL_KIND].trim());
    }

    private List<String> readLines() throws Exception {
        List<String> lines = new ArrayList<>();
        File file = csvFile();
        if (!file.exists()) return lines;
        try (BufferedReader in = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = in.readLine()) != null) {
                if (!line.trim().isEmpty()) lines.add(line);
            }
        }
        return lines;
    }

    /** Back up the file, then write the header, the old rows (minus removeRow, its last copy) and addRow (if any). */
    private void rewrite(String addRow, String removeRow) throws Exception {
        List<String> lines = readLines();
        File file = csvFile();
        if (file.exists()) {
            String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
            try (FileWriter bak = new FileWriter(new File(AppUtil.FIRST_FOLDER, CSV_NAME + BACKUP_TAG + stamp))) {
                for (String line : lines) bak.write(line + "\n");
            }
            pruneBackups();
        }
        if (removeRow != null) {
            int i = lines.lastIndexOf(removeRow);
            if (i < 0) throw new IllegalStateException("row not in the file any more");
            lines.remove(i);
        }
        try (FileWriter out = new FileWriter(file, false)) {
            out.write(CSV_HEADER + "\n");
            for (String line : lines) {
                if (!line.startsWith("time,")) out.write(line + "\n");   // the header is written above
            }
            if (addRow != null) out.write(addRow + "\n");
        }
    }

    /** Keep only the newest MAX_BACKUPS timestamped backups (the names sort oldest first). */
    private void pruneBackups() {
        File[] old = AppUtil.FIRST_FOLDER.listFiles((dir, name) -> name.startsWith(CSV_NAME + BACKUP_TAG));
        if (old == null || old.length <= MAX_BACKUPS) return;
        Arrays.sort(old);
        for (int i = 0; i < old.length - MAX_BACKUPS; i++) old[i].delete();
    }

    private void loadSaved() {
        saved.clear();
        try {
            for (String line : readLines()) {
                String[] f = line.split(",", -1);
                if (f.length >= 12 && !line.startsWith("time,")) saved.add(f);
            }
        } catch (Exception e) {
            lastSave = "READ FAILED: " + e;
        }
    }

    /** Save the last shot, where it was taken: as a best shot (feeds SHOT_TABLE), or as a compensation shot. */
    private void saveShot(boolean comp) {
        deleteArmed = false;
        if (lastShot == null) {
            lastSave = "no shot yet: shoot one ball first";
            return;
        }
        Shot s = lastShot;
        if (comp ? s.savedComp : s.savedBest) {
            lastSave = "this shot is already saved as " + (comp ? "comp" : "best") + ": shoot another";
            return;
        }
        String goal = goalName(robot.shooter.getGoal());
        String tof = Double.isNaN(s.tof) ? "" : String.format(Locale.US, "%.3f", s.tof);
        // set_dist and set_angle are left blank: the shot is calibrated where it was taken (x, y, dist).
        String row = String.format(Locale.US, "%s,%s,,,%.1f,%.1f,%.1f,%.1f,%.0f,%.1f,%s,%d,%s,%.0f,%.2f",
                new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date()),
                goal, s.x, s.y, s.headingDeg, s.dist, s.rpmSet, s.hood, tof,
                Double.isNaN(s.tof) ? 0 : 1, comp ? KIND_COMP : KIND_BEST, s.rpmMeas, s.volts);
        try {
            rewrite(row, null);
            saved.add(row.split(",", -1));
            sessionSaves.add(row);
            if (comp) s.savedComp = true; else s.savedBest = true;
            lastSave = String.format(Locale.US, "saved %s  %.1f in  rpm %.0f hood %.1f%s",
                    comp ? "COMP" : "best", s.dist, s.rpmSet, s.hood,
                    tof.isEmpty() ? "  (no flight time)" : "");
            gamepad1.rumble(200);
        } catch (Exception e) {
            lastSave = "SAVE FAILED: " + e;
        }
    }

    /** First press arms, a second press within DELETE_CONFIRM_S removes this session's last save. */
    private void undoLastSave() {
        if (sessionSaves.isEmpty()) {
            lastSave = "nothing saved this session to undo";
            deleteArmed = false;
            return;
        }
        if (!deleteArmed || deleteTimer.seconds() > DELETE_CONFIRM_S) {
            deleteArmed = true;
            deleteTimer.reset();
            return;
        }
        String row = sessionSaves.get(sessionSaves.size() - 1);
        try {
            rewrite(null, row);
            sessionSaves.remove(sessionSaves.size() - 1);
            for (int i = saved.size() - 1; i >= 0; i--) {
                if (String.join(",", saved.get(i)).equals(row)) { saved.remove(i); break; }
            }
            String[] f = row.split(",", -1);
            lastSave = String.format(Locale.US, "undone: %s  %s in  rpm %s hood %s",
                    isComp(f) ? "COMP" : "best", f[7], f[8], f[9]);
            gamepad1.rumbleBlips(2);
        } catch (Exception e) {
            lastSave = "UNDO FAILED: " + e;
        }
        deleteArmed = false;
    }

    /** Saved best shots within NEAR_DIST of a distance: "n (rpm/hood, ...)". */
    private String savedNear(double dist) {
        StringBuilder sb = new StringBuilder();
        int n = 0;
        for (String[] f : saved) {
            if (isComp(f)) continue;
            try {
                if (Math.abs(Double.parseDouble(f[7]) - dist) > NEAR_DIST) continue;
            } catch (NumberFormatException e) {
                continue;
            }
            if (n++ < 4) sb.append(sb.length() > 0 ? ", " : "").append(f[8]).append("/").append(f[9]);
        }
        return n == 0 ? "none" : n + "  (" + sb + (n > 4 ? ", ..." : "") + ")";
    }

    // == Telemetry ===========================================================

    private void showTelemetry() {
        Pose pose   = robot.drive.pose();
        Pose target = selectedPose();
        boolean driving = driveTo != null && driveTo.isScheduled();

        telemetry.addData("Goal", "%s (%.1f, %.1f)", goalName(robot.shooter.getGoal()),
                robot.shooter.getGoal().x(), robot.shooter.getGoal().y());
        telemetry.addData("Point", "%.0f in @ %+.0f deg -> (%.1f, %.1f) %s",
                DISTANCES[distIndex], ANGLES[angleIndex], target.x(), target.y(),
                !onField(target) ? "OFF FIELD" : driving ? "DRIVING" : "");
        telemetry.addData("Pose", "%5.1f, %5.1f, %5.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Distance : aim error", "%5.1f in : %5.1f deg  aim %s",
                robot.shooter.getDistance(), Math.toDegrees(robot.shooter.getAimError()),
                aiming ? "ON" : "off");
        double measured = robot.shooter.getMeasuredRPM();
        telemetry.addData("Flywheel RPM set : measured", "%5.0f : %5.0f  %s", flywheelOn ? rpm : 0, measured,
                !flywheelOn ? "OFF" : Math.abs(measured - rpm) < AT_SPEED_RPM ? "AT SPEED" : "spinning up");
        telemetry.addData("Hood (deg)", "%5.1f", hoodDeg);
        if (lastShot == null) {
            telemetry.addData("Last shot", "none yet: load ONE ball and shoot (RB)");
        } else {
            Shot s = lastShot;
            telemetry.addData("Last shot", "rpm %.0f (meas %.0f)  hood %.1f  %.1f V  tof %s%s",
                    s.rpmSet, s.rpmMeas, s.hood, s.volts,
                    Double.isNaN(s.tof) ? "none (LB when it goes in)" : String.format(Locale.US, "%.3f s", s.tof),
                    (s.rpmSet != (flywheelOn ? rpm : 0) || s.hood != hoodDeg) ? "   [settings changed since]" : "");
            telemetry.addData("  at", "%.1f, %.1f, %.1f deg  dist %.1f", s.x, s.y, s.headingDeg, s.dist);
        }
        telemetry.addData("Balls", robot.intake.ballCount());
        telemetry.addData("Best saved within " + (int) NEAR_DIST + " in", "%s",
                savedNear(robot.shooter.getDistance()));
        if (deleteArmed && deleteTimer.seconds() <= DELETE_CONFIRM_S) {
            telemetry.addLine(">>> UNDO the last save?  Press Options again <<<");
        }
        int comps = 0;
        for (String[] f : saved) if (isComp(f)) comps++;
        telemetry.addData("Saved", "%d best, %d comp  %s", saved.size() - comps, comps, lastSave);
        telemetry.update();
    }
}
