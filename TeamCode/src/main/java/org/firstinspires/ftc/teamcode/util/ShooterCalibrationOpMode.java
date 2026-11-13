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

import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

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
 *   Sticks              : drive (field centric, half speed); cancels a drive-to.  The chassis
 *                         aims at the goal whenever the right stick is idle.
 *   D-pad up / down     : select distance;  left / right : select angle off the goal's front
 *   X                   : drive to the selected point
 *   B                   : flywheel on / off
 *   Y / A               : RPM +/- 100
 *   Right / left trigger: hood +/- 1 degree
 *   Right bumper (hold) : shoot
 *   Left bumper         : ball went in (stops the flight timer for the oldest shot in the air)
 *   Share               : save this point
 *   Touchpad            : reset pose to CAL_START_POSE
 */
@TeleOp(name = "Shooter Calibration", group = "Tuning")
public class ShooterCalibrationOpMode extends CommandOpMode {

    // == Setup ===============================================================
    // TODO placeholder: a known, easy-to-reproduce starting spot (the Auto1 start position).
    private static final Pose CAL_START_POSE = new Pose(32.75, 6, Math.toRadians(180));

    // Calibration points: distance from the goal (inches) x angle off the goal's front (degrees,
    // + is CCW seen from above).  0 is straight in front of the goal.
    private static final double[] DISTANCES  = { 24, 36, 48, 60, 72, 96, 120 };
    private static final double[] ANGLES     = { -45, -30, 0, 30, 45 };

    // Pedro field direction (degrees) that each goal's front faces.  TODO measure on the field.
    private static final double AUDIENCE_GOAL_FACING_DEG = 270;
    private static final double FAR_GOAL_FACING_DEG      = 90;

    // Points closer than this to a wall (half a robot) are off the field.
    private static final double FIELD_MARGIN = 9;

    private static final String CSV_NAME = "shooter_cal.csv";
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
    private int     distIndex  = 2;
    private int     angleIndex = 2;
    private double  rpm        = 3000;
    private double  hoodDeg    = 12;
    private boolean flywheelOn = true;

    private FollowPathCommand driveTo;

    private final ElapsedTime        clock     = new ElapsedTime();
    private final ArrayDeque<Double> launches  = new ArrayDeque<>();   // launch times (s), oldest first
    private final List<Double>       flights   = new ArrayList<>();    // flight times for this point
    private int     lastBallCount = 0;
    private boolean lastRT, lastLT;

    private int     rowsSaved  = 0;
    private String  lastSave   = "";

    @Override
    public void initialize() {
        robot = new BiobuzzRobot(this);
        robot.initialize();
        robot.drive.setPose(CAL_START_POSE);
        robot.shooter.setAlliance(robot.config.red);
        applyShooter();

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
        pad.getGamepadButton(GamepadKeys.Button.BACK)
            .whenPressed(new InstantCommand(this::savePoint));
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

        if (driveTo != null && driveTo.isScheduled()) {
            if (!sticks) return;            // Pedro is driving
            driveTo.cancel();               // the driver takes over
        }
        driveTo = null;

        if (yaw == 0) {
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

    private void savePoint() {
        Pose pose = robot.drive.pose();
        Pose goal = robot.shooter.getGoal();
        String tof = flights.isEmpty() ? "" : String.format(Locale.US, "%.3f", meanFlight());
        String row = String.format(Locale.US, "%s,%s,%.0f,%.0f,%.1f,%.1f,%.1f,%.1f,%.0f,%.1f,%s,%d",
                new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date()),
                goalName(goal), DISTANCES[distIndex], ANGLES[angleIndex],
                pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                robot.shooter.getDistance(), rpm, hoodDeg, tof, flights.size());

        File file = new File(AppUtil.ROOT_FOLDER, CSV_NAME);
        boolean isNew = !file.exists();
        try (FileWriter out = new FileWriter(file, true)) {
            if (isNew) out.write(CSV_HEADER + "\n");
            out.write(row + "\n");
            rowsSaved++;
            lastSave = String.format(Locale.US, "saved %.0f in @ %.0f deg", DISTANCES[distIndex], ANGLES[angleIndex]);
            gamepad1.rumble(200);
        } catch (Exception e) {
            lastSave = "SAVE FAILED: " + e;
        }
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
        telemetry.addData("Distance : aim error", "%5.1f in : %5.1f deg",
                robot.shooter.getDistance(), Math.toDegrees(robot.shooter.getAimError()));
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
        telemetry.addData("Saved", "%d rows  %s", rowsSaved, lastSave);
        telemetry.update();
    }
}
