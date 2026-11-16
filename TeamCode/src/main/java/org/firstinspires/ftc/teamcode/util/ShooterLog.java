package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;

/**
 * Per-loop shooter log: one CSV row every robot loop, for the whole opmode run, to
 * /sdcard/FIRST/shooter_log/<OpMode>_<yyyyMMdd-HHmmss>.csv on the hub (the newest MAX_FILES are kept).
 * Pull and plot it on the PC with `python tools/shotlog.py`.
 *
 * Rows are buffered and flushed every FLUSH_EVERY_ROWS (about half a second), so logging doesn't
 * slow the loop; the last half second may be lost when the opmode stops.  Any file error stops the
 * log (see getStatus()) but never the robot.
 *
 * Columns (RPM = flywheel RPM; booleans 0/1):
 *   t_s, dt_ms        time since the first row, and since the previous row
 *   shooter_mode      IDLE / MANUAL / TRACKING
 *   target_rpm        speed wanted (shot table)
 *   command_rpm       speed asked of the motor this loop (target, or the spin-up ramp)
 *   actual_rpm        speed read this loop (the command while SHOOTER_SIMULATED)
 *   rpm_err           actual - target
 *   ramping, boosting spin-up ramp / full-power recovery (boost) active this loop
 *   at_speed, ready   isAtSpeed(), isReady()
 *   blocked           why not ready (blockedReason(), commas made ';'), empty when ready
 *   intake_mode       INTAKING / FULL / EJECTING / PASS / SPINUP / SHOOTING
 *   gate_pos          gate servo position commanded;  gate_open: it is the open position
 *   balls             ball count;  shoot_held: fire held
 *   dist_in, hood_deg, aim_err_deg   distance to the aim point, hood target, chassis aim error
 *   volts             battery, read once a second (it isn't in the bulk read)
 */
public class ShooterLog {

    public static final String FOLDER = "shooter_log";
    private static final int    MAX_FILES        = 10;
    private static final int    FLUSH_EVERY_ROWS = 25;
    private static final double VOLTS_EVERY_S    = 1.0;

    private static final String HEADER = "t_s,dt_ms,shooter_mode,target_rpm,command_rpm,actual_rpm,rpm_err,"
            + "ramping,boosting,at_speed,ready,blocked,intake_mode,gate_pos,gate_open,balls,shoot_held,"
            + "dist_in,hood_deg,aim_err_deg,volts";

    private final OpMode opMode;
    private BufferedWriter out;
    private String fileName = "";
    private String status   = "not started";
    private int    rows     = 0;

    private final ElapsedTime clock      = new ElapsedTime();
    private double            lastT      = -1;
    private VoltageSensor     battery;
    private double            volts      = 0;
    private double            lastVoltsT = -1e9;

    public ShooterLog(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Open a new log file for this opmode run (call once, during init). */
    public void start(String opModeName) {
        try {
            File dir = new File(AppUtil.FIRST_FOLDER, FOLDER);
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("can't create " + dir);
            fileName = opModeName + "_" + new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date()) + ".csv";
            out = new BufferedWriter(new FileWriter(new File(dir, fileName)));
            out.write(HEADER);
            out.newLine();
            pruneOldFiles(dir);
            battery = opMode.hardwareMap.voltageSensor.iterator().hasNext()
                    ? opMode.hardwareMap.voltageSensor.iterator().next() : null;
            status = "ok";
        } catch (IOException e) {
            fail(e);
        }
    }

    /** Write this loop's row (call once per loop, after the subsystems have run). */
    public void row(ShooterSubsystem shooter, IntakeSubsystem intake) {
        if (out == null) return;
        if (lastT < 0) clock.reset();               // t = 0 at the first row (START)
        double t  = clock.seconds();
        double dt = lastT < 0 ? 0 : (t - lastT) * 1000.0;
        lastT = t;
        if (battery != null && t - lastVoltsT >= VOLTS_EVERY_S) {
            volts = battery.getVoltage();
            lastVoltsT = t;
        }

        String blocked = shooter.blockedReason();
        blocked = blocked == null ? "" : blocked.replace(',', ';');
        double actual = shooter.getActualRPM();
        String line = String.format(Locale.US,
                "%.3f,%.1f,%s,%.0f,%.0f,%.0f,%.0f,%d,%d,%d,%d,%s,%s,%.3f,%d,%d,%d,%.1f,%.1f,%.1f,%.2f",
                t, dt, shooter.getMode(), shooter.getTargetRPM(), shooter.getCommandRPM(), actual,
                actual - shooter.getTargetRPM(),
                b(shooter.isRamping()), b(shooter.isBoosting()), b(shooter.isAtSpeed()), b(shooter.isReady()),
                blocked, intake.getMode(), intake.getGatePosition(), b(intake.isGateOpen()),
                intake.ballCount(), b(intake.isShooting()),
                shooter.getDistance(), shooter.getTargetHoodDeg(), Math.toDegrees(shooter.getAimError()),
                volts);
        try {
            out.write(line);
            out.newLine();
            if (++rows % FLUSH_EVERY_ROWS == 0) out.flush();
        } catch (IOException e) {
            fail(e);
        }
    }

    /** "shooter_log/<file> (n rows)", or why it isn't logging. */
    public String getStatus() {
        return "ok".equals(status) ? FOLDER + "/" + fileName + " (" + rows + " rows)" : status;
    }

    private static int b(boolean v) {
        return v ? 1 : 0;
    }

    private void fail(IOException e) {
        status = "LOG FAILED: " + e.getMessage();
        try {
            if (out != null) out.close();
        } catch (IOException ignored) {
        }
        out = null;
    }

    /** Keep only the newest MAX_FILES logs (the timestamped names sort oldest first per opmode, so sort by time). */
    private static void pruneOldFiles(File dir) {
        File[] files = dir.listFiles((d, name) -> name.endsWith(".csv"));
        if (files == null || files.length <= MAX_FILES) return;
        Arrays.sort(files, (a, c) -> Long.compare(a.lastModified(), c.lastModified()));
        for (int i = 0; i < files.length - MAX_FILES; i++) files[i].delete();
    }
}
