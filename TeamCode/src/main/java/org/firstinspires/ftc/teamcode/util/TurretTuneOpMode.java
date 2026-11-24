package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Turret control-loop tuner.  The robot runs the tests and the fitting; Panels
 * (http://192.168.43.1:8001) is the PC view: live graph of target / actual / power / chassis turn,
 * the results, and the gains (Configurables: TurretTuneConfig) editable live.  Nothing is saved:
 * when happy, paste the gains into the ShooterSubsystem TURRET_* constants.
 *
 * Before START: centre the turret by hand (pointing straight backwards), then press Cross to home.
 * Every test stays within WINDOW_DEG of where it started and inside the soft limits; Options aborts
 * (turret held where it is).  Start small (STEP_DEG, WINDOW_DEG) and raise them as the tuner proves itself.
 *
 *   Cross     Plant ID: a move check, the friction needed to break away (Ks, both ways), then short
 *             open-loop bursts that give the plant gain K (rad/s per power) and lag tau.  Ends with
 *             suggested Kp, Kd, Ks and Kv (= 1 / K).
 *   Square    Apply the suggested gains to the live ones (then run Triangle to see the effect).
 *   Triangle  Closed-loop step tests (+STEP, -STEP, +STEP, back) with the live gains: overshoot,
 *             rise, settle, final error.
 *   Circle    Kv test: the turret holds a fixed FIELD direction while you spin the chassis with the
 *             right stick; press Circle again to finish and get the Kv that cancels the residual
 *             error (Ki is off during it).
 *   Options   Abort.
 */
@Utility(name = "Turret Tune", description = "Turret PID / Kv tuning (Panels)")
public class TurretTuneOpMode extends CommandOpMode {

    private BiobuzzRobot robot;
    private ShooterSubsystem shooter;
    private VoltageSensor battery;

    private enum Stage { IDLE, PLANT, STEPS, KV }
    private Stage stage = Stage.IDLE;

    // Previous button states
    private boolean pCross, pSquare, pTriangle, pCircle, pOptions;

    private final ElapsedTime stageTimer = new ElapsedTime();   // since the current action began
    private final ElapsedTime loopTimer  = new ElapsedTime();
    private double loopS = 0.02, loopAvgS = 0.02;               // last / smoothed loop time
    private double angle, power, omega, headingUnwrapped, lastHeading;
    private double center;                                      // where the current test started

    private String status = "Home the turret in init (Cross), then press START";
    private final List<String> results = new ArrayList<>();

    // Suggested gains (from Plant ID and the Kv test)
    private double sKp = Double.NaN, sKi = 0, sKd = 0, sKs = 0, sKv = Double.NaN;

    // Recording of the current action
    private static final int MAX_SAMPLES = 2000;
    private final double[] rt = new double[MAX_SAMPLES], ra = new double[MAX_SAMPLES], rp = new double[MAX_SAMPLES];
    private int rn = 0;

    // == Plant ID ================================================================
    private static final double[] BURST_EXTRA = { 0.08, 0.16, 0.24 };   // power above Ks
    private static final double BREAK_RAMP_PER_S = 0.1;                 // breakaway power ramp
    private static final double BREAK_MOVE_RAD   = Math.toRadians(0.5);
    private int  plantAction;            // 0 check, 1 break +, 2 break -, 3.. bursts (+/- pairs), end
    private boolean returning;           // between actions: closed-loop back to the centre
    private double ksPos, ksNeg;
    private final double[] burstX = new double[6], burstY = new double[6], burstTau = new double[6];
    private int bursts;

    // == Steps ===================================================================
    private int stepIndex;
    private double stepFrom, stepTo, stepSize;
    private double[] stepTargets;
    private static final double STEP_HOLD_S = 1.2;
    private int stepsAnalysed;
    private double worstOvershoot, sumSettle, worstErr;
    private int settledCount;

    // == Kv test =================================================================
    private double kvTheta0, kvHeading0, kvKvUsed, kvKpUsed;
    private final double[] kvOmega = new double[6000], kvError = new double[6000];
    private int kvN;
    private static final double KV_MAX_S = 30;
    private static final double KV_MIN_OMEGA = 0.15;   // rad/s: ignore samples slower than this

    private TelemetryManager panels;

    @Override
    public void initialize() {
        robot = new BiobuzzRobot(this);
        robot.initialize(25);
        shooter = robot.shooter;
        shooter.idle();
        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        panels.setUpdateInterval(50);
        pushGains(TurretTuneConfig.KI);
        lastHeading = robot.drive.pose().heading();
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.cross && !pCross) {
            shooter.homeTurret();
            status = "Turret homed here (this is now 0).  Press START.";
        }
        pCross = gamepad1.cross;
        telemetry.addLine("Centre the turret by hand (pointing backwards), press Cross to home");
        telemetry.addLine(status);
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();                          // scheduler: the shooter reads the turret and sends its power

        loopS = Math.min(loopTimer.seconds(), 0.2);
        loopTimer.reset();
        loopAvgS += 0.05 * (loopS - loopAvgS);

        angle = shooter.getactualTurretRad();
        power = shooter.getTurretPower();
        double heading = robot.drive.pose().heading();
        headingUnwrapped += Angle.normalizeSigned(heading - lastHeading);
        lastHeading = heading;
        omega = robot.drive.velocity().omega;

        boolean cross = gamepad1.cross, square = gamepad1.square,
                triangle = gamepad1.triangle, circle = gamepad1.circle, options = gamepad1.options;

        if (options && !pOptions) abort("ABORTED");
        if (stage == Stage.IDLE) {
            if (cross && !pCross)       startPlant();
            if (triangle && !pTriangle) startSteps();
            if (circle && !pCircle)     startKv();
            if (square && !pSquare)     applySuggested();
        } else if (stage == Stage.KV && circle && !pCircle) {
            finishKv("done");
        }
        pCross = cross; pSquare = square; pTriangle = triangle; pCircle = circle; pOptions = options;

        // The gains: live from Panels.  Ki is off during the Kv test (it would hide the error).
        pushGains(stage == Stage.KV ? 0 : TurretTuneConfig.KI);

        if (stage != Stage.IDLE && outOfBounds()) {
            abort("ABORTED: left the safe window or a soft limit");
        }

        switch (stage) {
            case PLANT: runPlant(); break;
            case STEPS: runSteps(); break;
            case KV:    runKv();    break;
            default:    break;
        }

        robot.update();
        showTelemetry();
    }

    @Override
    public void end() {
        if (shooter != null) {
            shooter.setTurretOpenLoop(Double.NaN);
            shooter.setTurretKvAlways(false);
        }
    }

    // == Common ==================================================================

    private void pushGains(double ki) {
        shooter.setTurretGains(TurretTuneConfig.KP, ki, TurretTuneConfig.KD,
                               TurretTuneConfig.KS, TurretTuneConfig.KV);
    }

    private double windowRad() { return Math.toRadians(TurretTuneConfig.WINDOW_DEG); }

    /** Inside the soft limits (with a margin), and inside the window around the start (not for Kv). */
    private boolean outOfBounds() {
        double margin = Math.toRadians(5);
        if (angle < ShooterSubsystem.turretMinRad() + margin || angle > ShooterSubsystem.turretMaxRad() - margin) return true;
        return stage != Stage.KV && Math.abs(angle - center) > windowRad() * 1.3;   // 30% slack for overshoot
    }

    private void abort(String why) {
        shooter.setTurretOpenLoop(Double.NaN);
        shooter.setTurretKvAlways(false);
        shooter.holdTurretAt(angle);
        robot.drive.manual(ManualDrive.fieldCentric(0, 0, 0, 0));
        stage = Stage.IDLE;
        status = why;
    }

    private void begin(Stage s, String why) {
        results.clear();
        stage = s;
        center = angle;
        status = why;
        stageTimer.reset();
        rn = 0;
    }

    private void record() {
        if (rn < MAX_SAMPLES) {
            rt[rn] = stageTimer.seconds();
            ra[rn] = angle;
            rp[rn] = power;
            rn++;
        }
    }

    private void line(String s) { results.add(s); }

    private double volts() { return battery == null ? 0 : battery.getVoltage(); }

    // == Plant ID ================================================================

    private void startPlant() {
        begin(Stage.PLANT, "Plant ID: move check");
        plantAction = 0;
        returning   = false;
        bursts      = 0;
        beginAction();
    }

    private void beginAction() {
        stageTimer.reset();
        rn = 0;
        returning = false;
        center = angle;     // each action is centred on where the last return ended
        if (plantAction == 0)      status = "Plant ID: move check";
        else if (plantAction == 1) status = "Plant ID: breakaway +";
        else if (plantAction == 2) status = "Plant ID: breakaway -";
        else                       status = String.format(Locale.US, "Plant ID: burst %d/6", plantAction - 2);
    }

    private void finishAction() {
        shooter.setTurretOpenLoop(Double.NaN);
        shooter.holdTurretAt(center);        // closed loop (the live gains) back to the centre
        returning = true;
        stageTimer.reset();
    }

    private void runPlant() {
        if (returning) {
            shooter.holdTurretAt(center);
            if (Math.abs(angle - center) < Math.toRadians(1.0) && stageTimer.seconds() > 0.3
                    || stageTimer.seconds() > 2.5) {
                plantAction++;
                if (plantAction > 8) { finishPlant(); return; }
                beginAction();
            }
            return;
        }
        double t   = stageTimer.seconds();
        double d   = angle - center;
        double win = windowRad();

        if (plantAction == 0) {                              // does it move, and the right way?
            shooter.setTurretOpenLoop(TurretTuneConfig.CHECK_POWER);
            if (Math.abs(d) > Math.toRadians(2) || t > 0.4) {
                if (Math.abs(d) < Math.toRadians(0.5)) { failPlant("no movement at power " + TurretTuneConfig.CHECK_POWER); return; }
                if (d < 0) { failPlant("positive power moves the turret NEGATIVE: motor direction vs encoder is reversed"); return; }
                finishAction();
            }
        } else if (plantAction <= 2) {                       // breakaway: ramp the power until it moves
            double sign = plantAction == 1 ? 1 : -1;
            double u = sign * BREAK_RAMP_PER_S * t;
            shooter.setTurretOpenLoop(u);
            if (Math.abs(d) > BREAK_MOVE_RAD) {
                if (plantAction == 1) ksPos = Math.abs(u); else ksNeg = Math.abs(u);
                line(String.format(Locale.US, "Ks %s %.3f", plantAction == 1 ? "+" : "-", Math.abs(u)));
                finishAction();
            } else if (Math.abs(u) > 0.6 || Math.abs(d) > win * 0.7) {
                failPlant("no breakaway below 0.6 power");
            }
        } else {                                             // a burst just above Ks, one direction
            int k = plantAction - 3;
            double sign = (k % 2 == 0) ? 1 : -1;
            double ks   = sign > 0 ? ksPos : ksNeg;
            double u    = sign * (ks + BURST_EXTRA[k / 2]);
            shooter.setTurretOpenLoop(u);
            record();
            if (Math.abs(d) > win * 0.7 || t > 0.6) {
                analyseBurst(sign, Math.abs(u) - ks);
                finishAction();
            }
        }
    }

    private void analyseBurst(double sign, double x) {
        if (rn < 8) { line("burst too short to measure (reduce power or raise WINDOW_DEG)"); return; }
        double[] v = TurretTuneMath.speeds(rt, ra, rn, 3);
        // Steady speed: the mean over the last third of the burst.
        int from = rn * 2 / 3;
        double sum = 0;
        for (int i = from; i < rn; i++) sum += v[i];
        double vss = Math.abs(sum / (rn - from));
        if (vss < 0.05) { line("burst too slow to measure"); return; }
        double tau = Double.NaN;
        for (int i = 0; i < rn; i++) if (Math.abs(v[i]) >= 0.63 * vss) { tau = rt[i]; break; }
        if (bursts < burstX.length) {
            burstX[bursts] = x; burstY[bursts] = vss; burstTau[bursts] = tau;
            bursts++;
        }
        line(String.format(Locale.US, "burst %s %.2f above Ks: %.2f rad/s, %.0f ms to 63%% (%d samples)",
                sign > 0 ? "+" : "-", x, vss, tau * 1000, rn));
    }

    private void failPlant(String why) {
        abort("PLANT ID FAILED: " + why);
    }

    private void finishPlant() {
        shooter.setTurretOpenLoop(Double.NaN);
        shooter.holdTurretAt(center);
        stage = Stage.IDLE;
        if (bursts < 3) { status = "PLANT ID FAILED: too few usable bursts (raise WINDOW_DEG a little?)"; return; }
        double sxy = 0, sxx = 0;
        for (int i = 0; i < bursts; i++) { sxy += burstX[i] * burstY[i]; sxx += burstX[i] * burstX[i]; }
        double K   = sxy / sxx;                                  // rad/s per unit of power above Ks
        double tau = TurretTuneMath.median(burstTau, bursts);
        if (!(K > 0.1) || Double.isNaN(tau)) { status = "PLANT ID FAILED: bad fit (K " + K + ")"; return; }
        tau = Math.max(tau, 0.01);

        double[] pd = TurretTuneMath.designPd(K, tau, TurretTuneConfig.BANDWIDTH_RAD_S,
                                              TurretTuneConfig.DAMPING, loopAvgS);
        sKp = pd[0];
        sKd = pd[1];
        sKi = 0;
        sKs = 0.7 * (ksPos + ksNeg) / 2;      // a little under breakaway: enough to start moving, little hunting
        sKv = 1.0 / K;
        line(String.format(Locale.US, "Plant: K %.2f rad/s per power, tau %.0f ms, %.1f V, loop %.1f ms",
                K, tau * 1000, volts(), loopAvgS * 1000));
        line(String.format(Locale.US, "Design: w %.1f rad/s, zeta %.2f", pd[2], TurretTuneConfig.DAMPING));
        status = String.format(Locale.US,
                "PLANT ID DONE.  Suggest Kp %.2f Kd %.3f Ks %.3f Kv %.3f (Ki 0).  Square applies.",
                sKp, sKd, sKs, sKv);
    }

    private void applySuggested() {
        if (Double.isNaN(sKp)) { status = "Nothing to apply yet: run Plant ID (Cross)"; return; }
        TurretTuneConfig.KP = sKp;
        TurretTuneConfig.KI = sKi;
        TurretTuneConfig.KD = sKd;
        TurretTuneConfig.KS = sKs;
        TurretTuneConfig.KV = sKv;
        status = "Applied.  Triangle runs step tests with these gains.";
    }

    // == Closed-loop steps =======================================================

    private void startSteps() {
        begin(Stage.STEPS, "Steps");
        stepSize = Math.toRadians(Math.min(TurretTuneConfig.STEP_DEG, TurretTuneConfig.WINDOW_DEG / 2));
        stepTargets = new double[]{ center + stepSize, center - stepSize, center + stepSize, center };
        stepIndex = 0;
        stepsAnalysed = 0;
        worstOvershoot = 0; sumSettle = 0; worstErr = 0; settledCount = 0;
        stepFrom = center;
        stepTo   = stepTargets[0];
        shooter.holdTurretAt(stepTo);
        stageTimer.reset();
        rn = 0;
        status = "Steps 1/4";
    }

    private void runSteps() {
        shooter.holdTurretAt(stepTo);
        record();
        if (stageTimer.seconds() < STEP_HOLD_S) return;

        TurretTuneMath.StepResult r = TurretTuneMath.analyseStep(rt, ra, rp, rn, stepFrom, stepTo);
        line(String.format(Locale.US, "step %d (%+.1f deg): over %.0f%%  rise %s  settle %s  final err %+.2f deg  peak pwr %.2f",
                stepIndex + 1, r.sizeDeg, r.overshootPct,
                Double.isNaN(r.riseS) ? "-" : String.format(Locale.US, "%.0f ms", r.riseS * 1000),
                Double.isNaN(r.settleS) ? "NOT SETTLED" : String.format(Locale.US, "%.0f ms", r.settleS * 1000),
                r.finalErrDeg, r.peakPower));
        stepsAnalysed++;
        worstOvershoot = Math.max(worstOvershoot, r.overshootPct);
        worstErr = Math.max(worstErr, Math.abs(r.finalErrDeg));
        if (!Double.isNaN(r.settleS)) { sumSettle += r.settleS; settledCount++; }

        stepIndex++;
        if (stepIndex >= stepTargets.length) {
            stage = Stage.IDLE;
            StringBuilder hint = new StringBuilder();
            if (worstOvershoot > 15) hint.append("overshoot: lower Kp or raise Kd.  ");
            if (worstErr > 0.5)      hint.append("final error: raise Ks (or add a little Ki).  ");
            if (settledCount < stepsAnalysed) hint.append("a step never settled: check for hunting (lower Ks/Kp).  ");
            if (hint.length() == 0)  hint.append("looks good; try a bigger STEP_DEG.");
            status = String.format(Locale.US, "STEPS DONE: worst overshoot %.0f%%, mean settle %s, worst final err %.2f deg.  %s",
                    worstOvershoot,
                    settledCount > 0 ? String.format(Locale.US, "%.0f ms", 1000 * sumSettle / settledCount) : "-",
                    worstErr, hint);
            return;
        }
        stepFrom = stepTo;
        stepTo   = stepTargets[stepIndex];
        stageTimer.reset();
        rn = 0;
        status = String.format(Locale.US, "Steps %d/4", stepIndex + 1);
    }

    // == Kv ======================================================================

    private void startKv() {
        begin(Stage.KV, "Kv test: spin the chassis with the right stick (back and forth), Circle to finish");
        kvTheta0   = angle;
        kvHeading0 = headingUnwrapped;
        kvKvUsed   = TurretTuneConfig.KV;
        kvKpUsed   = TurretTuneConfig.KP;
        kvN = 0;
        shooter.setTurretKvAlways(true);
    }

    private void runKv() {
        // Hold a fixed field direction: the turret target moves opposite to the chassis heading.
        double target = kvTheta0 - (headingUnwrapped - kvHeading0);
        shooter.holdTurretAt(target);
        robot.drive.manual(ManualDrive.fieldCentric(0, 0, -gamepad1.right_stick_x * 0.6, 0));
        if (Math.abs(omega) > KV_MIN_OMEGA && kvN < kvOmega.length) {
            kvOmega[kvN] = omega;
            kvError[kvN] = target - angle;
            kvN++;
        }
        if (stageTimer.seconds() > KV_MAX_S) finishKv("time limit");
    }

    private void finishKv(String why) {
        shooter.setTurretKvAlways(false);
        shooter.holdTurretAt(angle);
        robot.drive.manual(ManualDrive.fieldCentric(0, 0, 0, 0));
        stage = Stage.IDLE;
        if (kvN < 40) { status = "Kv test (" + why + "): only " + kvN + " samples; spin the chassis more"; return; }
        double[] fit = TurretTuneMath.lineFit(kvOmega, kvError, kvN);
        // Steady error e = (Kv - 1/K) * omega / Kp, so the Kv that cancels it is Kv - Kp * slope.
        sKv = kvKvUsed - kvKpUsed * fit[0];
        line(String.format(Locale.US, "error vs turn rate: slope %.4f rad per rad/s, r2 %.2f, %d samples (Kv %.3f, Kp %.2f)",
                fit[0], fit[2], kvN, kvKvUsed, kvKpUsed));
        status = String.format(Locale.US, "KV DONE (%s): suggest Kv %.3f%s.  Square applies (with any Plant ID gains).",
                why, sKv, fit[2] < 0.3 ? "  (weak fit: spin faster / longer)" : "");
        if (Double.isNaN(sKp)) { sKp = kvKpUsed; sKd = TurretTuneConfig.KD; sKs = TurretTuneConfig.KS; sKi = TurretTuneConfig.KI; }
    }

    // == Telemetry ===============================================================

    private void showTelemetry() {
        telemetry.addData("Stage", "%s  %s", stage, status);
        telemetry.addData("Turret", "T %.1f A %.1f deg  pwr %.2f  omega %.2f rad/s",
                Math.toDegrees(shooter.getTurretTargetRad()), Math.toDegrees(angle), power, omega);
        telemetry.addData("Gains", "Kp %.3f Ki %.3f Kd %.3f Ks %.3f Kv %.3f",
                TurretTuneConfig.KP, TurretTuneConfig.KI, TurretTuneConfig.KD, TurretTuneConfig.KS, TurretTuneConfig.KV);
        if (!Double.isNaN(sKp)) {
            telemetry.addData("Suggest", "Kp %.3f Ki %.3f Kd %.3f Ks %.3f Kv %.3f", sKp, sKi, sKd, sKs, sKv);
        }
        for (int i = Math.max(0, results.size() - 4); i < results.size(); i++) telemetry.addLine(results.get(i));
        telemetry.addLine("Cross plant ID  Square apply  Triangle steps  Circle Kv  Options abort");
        telemetry.addData("Cycle", "%.1f ms", loopAvgS * 1000);
        telemetry.update();

        // Panels: numbers for the graph, then the results.
        panels.addData("stage", stage.toString());
        panels.addData("target_deg", Math.toDegrees(shooter.getTurretTargetRad()));
        panels.addData("actual_deg", Math.toDegrees(angle));
        panels.addData("power", power);
        panels.addData("omega", omega);
        panels.addData("volts", volts());
        panels.addData("status", status);
        for (String s : results) panels.addLine(s);
        if (!Double.isNaN(sKp)) {
            panels.addLine(String.format(Locale.US, "SUGGEST Kp %.4f Ki %.4f Kd %.4f Ks %.4f Kv %.4f", sKp, sKi, sKd, sKs, sKv));
        }
        panels.update();
    }
}
