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
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Shooter calibration: collects the data for ShooterSubsystem.SHOT_TABLE.
 *
 * Pick a calibration point (a distance and an angle off the front of the goal on the robot's side),
 * drive there (by hand or with X), load ONE ball, and shoot it (RB) at the RPM and hood you set.
 * Press LB when it enters the goal to time its flight.  Repeat, adjusting RPM and hood, until the
 * shots are consistent, then save.  What is saved is the LAST SHOT (its RPM and hood as they were
 * when it left, the flywheel speed measured then, battery volts, flight time, field position), not
 * the current settings, so you can nudge things after the shot without spoiling the data.
 *
 *   Share      saves it as the best shot for the point (feeds SHOT_TABLE; replaces an earlier one)
 *   Left stick press saves it as a COMPENSATION shot: an intentionally different RPM (normally lower)
 *              with the hood that works for it.  Several per point (one per RPM and hood pair, so a
 *              sweep of both is kept; the same pair replaces).  Not used for SHOT_TABLE; only for working out hood compensation.
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
 *   Share               : save the last shot as the best shot for this point
 *   Left stick press    : save the last shot as a compensation shot
 *   Options (twice)     : delete the best shot saved for the selected point
 *   Touchpad            : reset pose to CAL_START_POSE
 *
 * Selecting a different point, or driving to one, forgets the last shot.
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

    // Aiming (same as teleop)
    private static final double AIM_GAIN     = 0.5;
    private static final double MAX_AIM_TURN = 0.6;
    private static final double DRIVE_SCALE  = 0.5;

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

    /** Everything about the last shot, taken as it left; this is what gets saved. */
    private static class Shot {
        double launchT;                 // clock time of the launch (s)
        double rpmSet, hood;            // the settings it was shot with
        double rpmMeas;                 // measured flywheel speed as it left
        double volts;
        double x, y, headingDeg, dist;  // robot pose and distance to the goal
        double tof = Double.NaN;        // flight time, once LB is pressed
    }

    private final ElapsedTime        clock     = new ElapsedTime();
    private final ArrayDeque<double[]> rpmHistory = new ArrayDeque<>();   // { time (s), measured rpm }
    private VoltageSensor battery;
    private Shot    lastShot;                   // null until a shot at the selected point
    private int     lastBallCount = 0;
    private boolean lastRT, lastLT;

    // Saved points from the file: "goal,dist,angle" -> that point's latest row (CSV fields).
    private final Map<String, String[]> saved = new HashMap<>();
    private String  lastSave     = "";
    private boolean deleteArmed  = false;          // Options pressed once; press again to delete
    private final ElapsedTime deleteTimer = new ElapsedTime();
    private static final double DELETE_CONFIRM_S = 2.0;

    @Override
    public void initialize() {
        robot = new BiobuzzRobot(this);
        robot.initialize();
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
            .whenPressed(new InstantCommand(() -> adjust(100, 0)));
        pad.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(() -> adjust(-100, 0)));
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
            .whenPressed(new InstantCommand(this::deletePoint));
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
            }
            driveTo = null;
        }

        if (yaw == 0 && aiming) {
            yaw = Range.clip(robot.shooter.getAimError() * AIM_GAIN, -MAX_AIM_TURN, MAX_AIM_TURN);
        }
        DrivePowers powers = ManualDrive.fieldCentric(axial, lateral, yaw, robot.drive.pose().heading());
        robot.drive.manual(powers);
    }

    private void driveToPoint() {
        Pose target = selectedPose();
        if (!onField(target)) return;
        if (driveTo != null) driveTo.cancel();
        clearShot();
        Pose start = robot.drive.pose();
        Path path = Paths.path(line(start, target).linear(start, target));
        driveTo = new FollowPathCommand(robot.drive, path);
        schedule(driveTo);
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
        clearShot();
    }

    // == Shooter =============================================================

    private void adjust(double dRpm, double dHoodDeg) {
        rpm     = Range.clip(rpm + dRpm, 0, 6000);
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

    private void clearShot() {
        lastShot = null;
    }

    // == Saving ==============================================================
    // Each point is saved once: saving again replaces it, and Options (twice) deletes it.  Before
    // every change the old file is copied to shooter_cal.csv.bak-<time> (the newest 5 are kept), so mistakes can be undone.

    private File csvFile() {
        return new File(AppUtil.FIRST_FOLDER, CSV_NAME);
    }

    private static String key(String goal, double dist, double angle) {
        return String.format(Locale.US, "%s,%.0f,%.0f", goal, dist, angle);
    }

    /** A compensation shot is one per (goal, distance, angle, RPM, hood): a sweep of both is kept. */
    private static String compKey(String goal, double dist, double angle, double rpm, double hood) {
        return String.format(Locale.US, "comp,%s,%.0f,%.0f,%.0f,%.1f", goal, dist, angle, rpm, hood);
    }

    private static String key(String[] f) {
        try {
            String goal  = f[1].trim();
            double dist  = Double.parseDouble(f[2]);
            double angle = Double.parseDouble(f[3]);
            boolean comp = f.length > COL_KIND && KIND_COMP.equals(f[COL_KIND].trim());
            return comp ? compKey(goal, dist, angle, Double.parseDouble(f[8]), Double.parseDouble(f[9])) : key(goal, dist, angle);
        } catch (Exception e) {
            return null;                    // header or a bad line
        }
    }

    private String selectedKey() {
        return key(goalName(robot.shooter.getGoal()), DISTANCES[distIndex], ANGLES[angleIndex]);
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

    /** Rewrite the file: the header, then every row except the one with key replaceKey, then newRow (if any). */
    private void rewrite(String newRow, String replaceKey) throws Exception {
        List<String> lines = readLines();
        String sel = replaceKey;
        File file = csvFile();
        if (file.exists()) {
            String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
            try (FileWriter bak = new FileWriter(new File(AppUtil.FIRST_FOLDER, CSV_NAME + BACKUP_TAG + stamp))) {
                for (String line : lines) bak.write(line + "\n");
            }
            pruneBackups();
        }
        try (FileWriter out = new FileWriter(file, false)) {
            out.write(CSV_HEADER + "\n");
            for (String line : lines) {
                String k = key(line.split(",", -1));
                if (k == null) {
                    if (!line.startsWith("time,")) out.write(line + "\n");   // keep odd lines; the header is rewritten above
                } else if (!k.equals(sel)) {
                    out.write(line + "\n");
                }
            }
            if (newRow != null) out.write(newRow + "\n");
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
                String k = key(f);
                if (k != null && f.length >= 12) saved.put(k, f);
            }
        } catch (Exception e) {
            lastSave = "READ FAILED: " + e;
        }
    }

    /** Save the last shot: as the point's best shot (feeds SHOT_TABLE), or as a compensation shot. */
    private void saveShot(boolean comp) {
        deleteArmed = false;
        if (lastShot == null) {
            lastSave = "no shot at this point yet: shoot one ball first";
            return;
        }
        Shot s = lastShot;
        String goal = goalName(robot.shooter.getGoal());
        String saveKey = comp ? compKey(goal, DISTANCES[distIndex], ANGLES[angleIndex], s.rpmSet, s.hood)
                              : selectedKey();
        String tof = Double.isNaN(s.tof) ? "" : String.format(Locale.US, "%.3f", s.tof);
        String row = String.format(Locale.US, "%s,%s,%.0f,%.0f,%.1f,%.1f,%.1f,%.1f,%.0f,%.1f,%s,%d,%s,%.0f,%.2f",
                new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date()),
                goal, DISTANCES[distIndex], ANGLES[angleIndex],
                s.x, s.y, s.headingDeg, s.dist, s.rpmSet, s.hood, tof,
                Double.isNaN(s.tof) ? 0 : 1, comp ? KIND_COMP : KIND_BEST, s.rpmMeas, s.volts);
        try {
            boolean replaced = saved.containsKey(saveKey);
            rewrite(row, saveKey);
            saved.put(saveKey, row.split(",", -1));
            lastSave = String.format(Locale.US, "%s %s %.0f in @ %.0f deg  rpm %.0f hood %.1f%s",
                    replaced ? "replaced" : "saved", comp ? "COMP" : "best",
                    DISTANCES[distIndex], ANGLES[angleIndex], s.rpmSet, s.hood,
                    tof.isEmpty() ? "  (no flight time)" : "");
            gamepad1.rumble(200);
        } catch (Exception e) {
            lastSave = "SAVE FAILED: " + e;
        }
    }

    /** First press arms, a second press within DELETE_CONFIRM_S deletes the selected point. */
    private void deletePoint() {
        if (!saved.containsKey(selectedKey())) {
            lastSave = "nothing saved at this point";
            deleteArmed = false;
            return;
        }
        if (!deleteArmed || deleteTimer.seconds() > DELETE_CONFIRM_S) {
            deleteArmed = true;
            deleteTimer.reset();
            return;
        }
        try {
            rewrite(null, selectedKey());
            saved.remove(selectedKey());
            lastSave = String.format(Locale.US, "deleted %.0f in @ %.0f deg", DISTANCES[distIndex], ANGLES[angleIndex]);
            gamepad1.rumbleBlips(2);
        } catch (Exception e) {
            lastSave = "DELETE FAILED: " + e;
        }
        deleteArmed = false;
    }

    /** "3000/12.0, 2700/14.5" : the compensation shots saved at the selected point (rpm/hood). */
    private String compSummary() {
        String prefix = String.format(Locale.US, "comp,%s,%.0f,%.0f,",
                goalName(robot.shooter.getGoal()), DISTANCES[distIndex], ANGLES[angleIndex]);
        List<String[]> rows = new ArrayList<>();
        for (Map.Entry<String, String[]> e : saved.entrySet()) {
            if (e.getKey().startsWith(prefix)) rows.add(e.getValue());
        }
        if (rows.isEmpty()) return "none";
        StringBuilder sb = new StringBuilder();
        for (String[] r : rows) sb.append(sb.length() > 0 ? ", " : "").append(r[8]).append("/").append(r[9]);
        return sb.toString();
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
        telemetry.addData("RPM set : measured", "%5.0f : %5.0f  %s", flywheelOn ? rpm : 0, measured,
                !flywheelOn ? "OFF" : Math.abs(measured - rpm) < AT_SPEED_RPM ? "AT SPEED" : "spinning up");
        telemetry.addData("Hood (deg)", "%5.1f", hoodDeg);
        if (lastShot == null) {
            telemetry.addData("Last shot", "none at this point: load ONE ball and shoot (RB)");
        } else {
            Shot s = lastShot;
            telemetry.addData("Last shot", "rpm %.0f (meas %.0f)  hood %.1f  %.1f V  tof %s%s",
                    s.rpmSet, s.rpmMeas, s.hood, s.volts,
                    Double.isNaN(s.tof) ? "none (LB when it goes in)" : String.format(Locale.US, "%.3f s", s.tof),
                    (s.rpmSet != (flywheelOn ? rpm : 0) || s.hood != hoodDeg) ? "   [settings changed since]" : "");
            telemetry.addData("  at", "%.1f, %.1f, %.1f deg  dist %.1f", s.x, s.y, s.headingDeg, s.dist);
        }
        telemetry.addData("Balls", robot.intake.ballCount());
        String[] here = saved.get(selectedKey());
        telemetry.addData("Best saved here", here == null ? "no" : String.format(Locale.US,
                "rpm %s  hood %s  tof %s  (%s)", here[8], here[9],
                here[10].isEmpty() ? "-" : here[10], here[0]));
        telemetry.addData("Comp saved here", "%s", compSummary());
        if (deleteArmed && deleteTimer.seconds() <= DELETE_CONFIRM_S) {
            telemetry.addLine(">>> DELETE the best shot here?  Press Options again <<<");
        }
        int comps = 0;
        for (String k : saved.keySet()) if (k.startsWith("comp,")) comps++;
        telemetry.addData("Saved", "%d best, %d comp  %s", saved.size() - comps, comps, lastSave);
        telemetry.update();
    }
}
