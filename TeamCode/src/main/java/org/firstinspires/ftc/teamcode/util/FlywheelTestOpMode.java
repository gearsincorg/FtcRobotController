package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.FLYWHEEL_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.rpmToTicksPerSec;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ticksPerSecToRpm;

/**
 * Flywheel test: checks that the "shooter" motor holds the RPM it is asked for.
 * The motor is set up the same way as ShooterSubsystem (REVERSE, FLOAT, same ticks/rev).
 * Keep hands and balls clear of the flywheel.
 *
 *   A           open-loop sweep: setPower 0.2..1.0, read the speed.  Full power gives the free
 *               speed (checks FLYWHEEL_TICKS_PER_REV) and a suggested PIDF (F = 32767 / max ticks/s,
 *               P = 0.1 F, I = 0.1 P)
 *   B           closed-loop steps: setVelocity at each STEP_RPMS, reports actual RPM, error,
 *               ripple and time to get within READY_TOL_RPM
 *   X           apply the suggested PIDF (run A first); B again to compare
 *   Y           hold / stop the manual target RPM
 *   Dpad up/dn  manual target RPM +/- 100
 *   LB / RB     P -/+ 1        Dpad left/right  F -/+ 0.5   (live, while holding)
 *
 * Results stay on screen and are written to FIRST/flywheel_test.txt on the hub.
 */
@TeleOp(name = "Flywheel Test", group = "Tuning")
public class FlywheelTestOpMode extends LinearOpMode {

    private static final double[] SWEEP_POWERS  = { 0.2, 0.4, 0.6, 0.8, 1.0 };
    private static final double[] STEP_RPMS     = { 1000, 2000, 2400, 2800, 3200, 3600, 4000, 4500, 5000 };
    private static final double   SETTLE_S      = 1.5;   // ignore readings while the speed settles
    private static final double   SAMPLE_S      = 1.5;   // then average over this long
    private static final double   READY_TOL_RPM = 100;

    private DcMotorEx     flywheel;
    private VoltageSensor battery;
    private final List<String> log = new ArrayList<>();
    private double  maxTps    = 0;      // measured at full power
    private double  targetRpm = 3000;
    private boolean holding   = false;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        battery = hardwareMap.voltageSensor.iterator().next();

        line("Motor type: " + flywheel.getMotorType().getName()
                + String.format(Locale.US, " (%.1f ticks/rev, %.0f rpm)",
                    flywheel.getMotorType().getTicksPerRev(), flywheel.getMotorType().getMaxRPM()));
        line(String.format(Locale.US, "Code ticks/rev: %.1f", FLYWHEEL_TICKS_PER_REV));
        line("Default PIDF: " + pidfText());

        while (opModeInInit()) {
            showLog("Press START, then A (sweep) or B (steps)");
        }

        Gamepad prev = new Gamepad();
        Gamepad now  = new Gamepad();
        while (opModeIsActive()) {
            prev.copy(now);
            now.copy(gamepad1);

            if (now.a && !prev.a) runSweep();
            if (now.b && !prev.b) runSteps();
            if (now.x && !prev.x) applySuggested();
            if (now.y && !prev.y) holding = !holding;
            if (now.dpad_up && !prev.dpad_up)     targetRpm += 100;
            if (now.dpad_down && !prev.dpad_down) targetRpm = Math.max(0, targetRpm - 100);
            if (now.right_bumper && !prev.right_bumper) adjustPidf(1, 0);
            if (now.left_bumper && !prev.left_bumper)   adjustPidf(-1, 0);
            if (now.dpad_right && !prev.dpad_right)     adjustPidf(0, 0.5);
            if (now.dpad_left && !prev.dpad_left)       adjustPidf(0, -0.5);

            if (holding) {
                flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flywheel.setVelocity(rpmToTicksPerSec(targetRpm));
            } else {
                flywheel.setPower(0);
            }

            telemetry.addData("Manual", "%s  target %5.0f  actual %5.0f rpm  %.2f V",
                    holding ? "HOLD" : "off", targetRpm, ticksPerSecToRpm(flywheel.getVelocity()),
                    battery.getVoltage());
            telemetry.addData("PIDF", pidfText());
            showLog("A sweep  B steps  X apply suggested  Y hold");
        }
        flywheel.setPower(0);
    }

    // == Tests ================================================================

    private void runSweep() {
        holding = false;
        line("");
        line("OPEN LOOP  power   rpm   ticks/s  +/-rpm  volts");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        for (double p : SWEEP_POWERS) {
            if (!opModeIsActive()) break;
            flywheel.setPower(p);
            double[] m = measure(String.format(Locale.US, "sweep power %.1f", p), Double.NaN);
            line(String.format(Locale.US, "          %4.2f  %5.0f  %6.0f   %5.0f  %5.2f",
                    p, ticksPerSecToRpm(m[0]), m[0], ticksPerSecToRpm(m[1]), m[3]));
            if (p >= 1.0) maxTps = m[0];
        }
        flywheel.setPower(0);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (maxTps > 0) {
            double f = 32767 / maxTps;
            line(String.format(Locale.US, "Free speed %.0f rpm.  Suggested PIDF  P %.2f  I %.3f  D 0  F %.2f",
                    ticksPerSecToRpm(maxTps), 0.1 * f, 0.01 * f, f));
        }
        saveLog();
    }

    private void runSteps() {
        holding = false;
        line("");
        line("CLOSED LOOP  PIDF " + pidfText());
        line("  target  actual   err  +/-rpm  reach(s)  volts");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for (double rpm : STEP_RPMS) {
            if (!opModeIsActive()) break;
            double tps = rpmToTicksPerSec(rpm);
            if (maxTps > 0 && tps > 0.95 * maxTps) {
                line(String.format(Locale.US, "  %5.0f   skipped: above 95%% of free speed", rpm));
                continue;
            }
            flywheel.setVelocity(tps);
            double[] m = measure(String.format(Locale.US, "step %.0f rpm", rpm), tps);
            double actual = ticksPerSecToRpm(m[0]);
            line(String.format(Locale.US, "  %5.0f   %5.0f  %+5.0f   %5.0f   %s  %5.2f",
                    rpm, actual, actual - rpm, ticksPerSecToRpm(m[1]),
                    m[2] < 0 ? " never " : String.format(Locale.US, "%6.2f ", m[2]), m[3]));
        }
        flywheel.setPower(0);
        saveLog();
    }

    /**
     * Waits SETTLE_S, then averages for SAMPLE_S.
     * Returns { mean ticks/s, std dev ticks/s, seconds to get within READY_TOL_RPM (-1 = never), volts }.
     */
    private double[] measure(String label, double targetTps) {
        ElapsedTime t = new ElapsedTime();
        double sum = 0, sumSq = 0, volts = 0, reach = -1;
        int n = 0;
        while (opModeIsActive() && t.seconds() < SETTLE_S + SAMPLE_S) {
            double tps = flywheel.getVelocity();
            if (!Double.isNaN(targetTps) && reach < 0
                    && Math.abs(ticksPerSecToRpm(tps - targetTps)) < READY_TOL_RPM) {
                reach = t.seconds();
            }
            if (t.seconds() > SETTLE_S) {
                sum += tps;
                sumSq += tps * tps;
                volts += battery.getVoltage();
                n++;
            }
            telemetry.addData("Running", label);
            telemetry.addData("rpm now", "%5.0f", ticksPerSecToRpm(tps));
            showLog("");
        }
        if (n == 0) return new double[] { 0, 0, reach, 0 };
        double mean = sum / n;
        return new double[] { mean, Math.sqrt(Math.max(0, sumSq / n - mean * mean)), reach, volts / n };
    }

    // == PIDF =================================================================

    private void applySuggested() {
        if (maxTps <= 0) {
            line("Run the sweep (A) before applying a suggested PIDF");
            return;
        }
        double f = 32767 / maxTps;
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(0.1 * f, 0.01 * f, 0, f));
        line("Applied PIDF " + pidfText());
    }

    private void adjustPidf(double dP, double dF) {
        PIDFCoefficients c = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(Math.max(0, c.p + dP), c.i, c.d, Math.max(0, c.f + dF)));
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(3,0.5,0,10.0));
    }

    private String pidfText() {
        PIDFCoefficients c = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        return String.format(Locale.US, "P %.2f  I %.3f  D %.2f  F %.2f", c.p, c.i, c.d, c.f);
    }

    // == Output ===============================================================

    private void line(String s) {
        log.add(s);
    }

    private void showLog(String help) {
        if (!help.isEmpty()) telemetry.addLine(help);
        for (String s : log) telemetry.addLine(s);
        telemetry.update();
    }

    private void saveLog() {
        File file = new File(AppUtil.FIRST_FOLDER, "flywheel_test.txt");
        ReadWriteFile.writeFile(file, String.join("\n", log) + "\n");
        line("Saved " + file.getPath());
    }
}
