package org.firstinspires.ftc.teamcode.util;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 * drive there (by hand or with X), adjust RPM and hood until shots go in, press LB each time a
 * ball enters the goal to time its flight, then Share to save the point.  Points are appended to
 * /sdcard/FIRST/shooter_cal.csv on the hub.  On the PC, `python tools/shotcal.py` pulls that file
 * and rewrites SHOT_TABLE in ShooterSubsystem.java.
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
 *   Left bumper         : ball went in (stops the flight timer for the oldest shot in the air)
 *   Share               : save this point (replaces an earlier save of the same point)
 *   Options (twice)     : delete the saved data for the selected point
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
    private static final String CSV_HEADER =
            "time,goal,set_dist,set_angle,x,y,heading_deg,dist,rpm,hood_deg,tof_s,tof_n";

    // Aiming (same as teleop)
    private static final double AIM_GAIN     = 0.5;
    private static final double MAX_AIM_TURN = 0.6;
    private static final double DRIVE_SCALE  = 0.5;

    // Flight time: an LB press more than this long after a launch doesn't count.
    private static final double MAX_FLIGHT_S = 2.0;
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

    private final ElapsedTime        clock     = new ElapsedTime();
    private final ArrayDeque<Double> launches  = new ArrayDeque<>();   // launch times (s), oldest first
    private final List<Double>       flights   = new ArrayList<>();    // flight times for this point
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
            .whenPressed(new InstantCommand(this::savePoint));
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
        resetFlights();
    }

    // == Shooter =============================================================

    private void adjust(double dRpm, double dHoodDeg) {
        rpm     = Range.clip(rpm + dRpm, 0, 6000);
        hoodDeg = Range.clip(hoodDeg + dHoodDeg, ShooterSubsystem.HOOD_MIN_DEG, ShooterSubsystem.HOOD_MAX_DEG);
        applyShooter();
        resetFlights();
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

    /** A ball left when the count drops while shooting; the count lags by the sensor debounce. */
    private void watchLaunches() {
        int count = robot.intake.ballCount();
        double now = clock.seconds();
        if (count < lastBallCount && robot.intake.getMode() == IntakeSubsystem.Mode.SHOOTING) {
            for (int i = count; i < lastBallCount; i++) {
                launches.addLast(now - IntakeSubsystem.COUNT_DEBOUNCE_MS / 1000.0);
            }
        }
        lastBallCount = count;
        while (!launches.isEmpty() && now - launches.peekFirst() > MAX_FLIGHT_S) {
            launches.removeFirst();         // missed, or nobody pressed LB
        }
    }

    private void ballIn() {
        if (launches.isEmpty()) return;
        flights.add(clock.seconds() - launches.removeFirst());
    }

    private void resetFlights() {
        flights.clear();
        launches.clear();
    }

    private double meanFlight() {
        double sum = 0;
        for (double f : flights) sum += f;
        return flights.isEmpty() ? 0 : sum / flights.size();
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

    private static String key(String[] f) {
        try {
            return key(f[1].trim(), Double.parseDouble(f[2]), Double.parseDouble(f[3]));
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

    /** Rewrite the file: the header, then every row except the selected point's, then newRow (if any). */
    private void rewrite(String newRow) throws Exception {
        List<String> lines = readLines();
        String sel = selectedKey();
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

    private void savePoint() {
        Pose pose = robot.drive.pose();
        Pose goal = robot.shooter.getGoal();
        String tof = flights.isEmpty() ? "" : String.format(Locale.US, "%.3f", meanFlight());
        String row = String.format(Locale.US, "%s,%s,%.0f,%.0f,%.1f,%.1f,%.1f,%.1f,%.0f,%.1f,%s,%d",
                new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date()),
                goalName(goal), DISTANCES[distIndex], ANGLES[angleIndex],
                pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                robot.shooter.getDistance(), rpm, hoodDeg, tof, flights.size());
        try {
            boolean replaced = saved.containsKey(selectedKey());
            rewrite(row);
            saved.put(selectedKey(), row.split(",", -1));
            lastSave = String.format(Locale.US, "%s %.0f in @ %.0f deg", replaced ? "replaced" : "saved",
                    DISTANCES[distIndex], ANGLES[angleIndex]);
            gamepad1.rumble(200);
        } catch (Exception e) {
            lastSave = "SAVE FAILED: " + e;
        }
        deleteArmed = false;
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
            rewrite(null);
            saved.remove(selectedKey());
            lastSave = String.format(Locale.US, "deleted %.0f in @ %.0f deg", DISTANCES[distIndex], ANGLES[angleIndex]);
            gamepad1.rumbleBlips(2);
        } catch (Exception e) {
            lastSave = "DELETE FAILED: " + e;
        }
        deleteArmed = false;
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
        if (flights.isEmpty()) {
            telemetry.addData("Flight time", "none (LB when a ball goes in)  in air: %d", launches.size());
        } else {
            double lo = Double.MAX_VALUE, hi = 0;
            for (double f : flights) { lo = Math.min(lo, f); hi = Math.max(hi, f); }
            telemetry.addData("Flight time", "%.3f s  n=%d  range %.3f-%.3f  in air: %d",
                    meanFlight(), flights.size(), lo, hi, launches.size());
        }
        telemetry.addData("Balls", robot.intake.ballCount());
        String[] here = saved.get(selectedKey());
        telemetry.addData("Saved here", here == null ? "no" : String.format(Locale.US,
                "rpm %s  hood %s  tof %s  (%s)", here[8], here[9],
                here[10].isEmpty() ? "-" : here[10], here[0]));
        if (deleteArmed && deleteTimer.seconds() <= DELETE_CONFIRM_S) {
            telemetry.addLine(">>> DELETE this point?  Press Options again <<<");
        }
        telemetry.addData("Saved points", "%d  %s", saved.size(), lastSave);
        telemetry.update();
    }
}
