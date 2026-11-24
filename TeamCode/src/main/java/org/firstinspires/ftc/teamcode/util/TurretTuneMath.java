package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.Arrays;

/** The arithmetic for TurretTuneOpMode: line fits, step-response metrics and the gain design. */
public final class TurretTuneMath {
    private TurretTuneMath() {}

    /** Least-squares line y = a + b x over the first n points: {slope b, intercept a, r2}. */
    public static double[] lineFit(double[] x, double[] y, int n) {
        if (n < 2) return new double[]{0, 0, 0};
        double sx = 0, sy = 0;
        for (int i = 0; i < n; i++) { sx += x[i]; sy += y[i]; }
        double mx = sx / n, my = sy / n, sxx = 0, sxy = 0, syy = 0;
        for (int i = 0; i < n; i++) {
            sxx += (x[i] - mx) * (x[i] - mx);
            sxy += (x[i] - mx) * (y[i] - my);
            syy += (y[i] - my) * (y[i] - my);
        }
        if (sxx < 1e-12) return new double[]{0, my, 0};
        double slope = sxy / sxx;
        double r2    = syy < 1e-12 ? 0 : (sxy * sxy) / (sxx * syy);
        return new double[]{slope, my - slope * mx, r2};
    }

    public static double median(double[] v, int n) {
        if (n == 0) return Double.NaN;
        double[] c = Arrays.copyOf(v, n);
        Arrays.sort(c);
        return n % 2 == 1 ? c[n / 2] : 0.5 * (c[n / 2 - 1] + c[n / 2]);
    }

    /**
     * Speed (rad/s) from an angle record: for each sample the change over the last `span` samples
     * divided by the time (the encoder is coarse, so single-loop differences are noisy).
     */
    public static double[] speeds(double[] t, double[] angle, int n, int span) {
        double[] v = new double[n];
        for (int i = 0; i < n; i++) {
            int j = Math.max(0, i - span);
            double dt = t[i] - t[j];
            v[i] = dt > 1e-4 ? (angle[i] - angle[j]) / dt : 0;
        }
        return v;
    }

    /** What one closed-loop step looked like. */
    public static class StepResult {
        public double sizeDeg, overshootPct, riseS = Double.NaN, settleS = Double.NaN,
                      finalErrDeg, peakPower;
    }

    /**
     * Analyse a step from `from` to `to` (radians): t is seconds since the step, angle the turret
     * angle, power the power sent, n samples.  Settled = within max(2% of the step, the Ks deadband)
     * from then on: the deadband is just over half an encoder tick, so resting on the tick nearest
     * the target counts as settled (a tighter band can't be met when the target falls between ticks).
     */
    public static StepResult analyseStep(double[] t, double[] angle, double[] power, int n, double from, double to) {
        StepResult r = new StepResult();
        double size = to - from;
        r.sizeDeg = Math.toDegrees(size);
        if (n < 3 || Math.abs(size) < 1e-6) return r;
        double band = Math.max(0.02 * Math.abs(size), ShooterSubsystem.TURRET_KS_DEADBAND_RAD);
        double peak = 0, t10 = Double.NaN, t90 = Double.NaN;
        double lastOutside = 0;
        for (int i = 0; i < n; i++) {
            double y = (angle[i] - from) / size;          // 0 at the start, 1 at the target
            peak = Math.max(peak, y);
            if (Double.isNaN(t10) && y >= 0.1) t10 = t[i];
            if (Double.isNaN(t90) && y >= 0.9) t90 = t[i];
            if (Math.abs(angle[i] - to) > band) lastOutside = t[i];
            r.peakPower = Math.max(r.peakPower, Math.abs(power[i]));
        }
        r.overshootPct = Math.max(0, peak - 1) * 100;
        if (!Double.isNaN(t10) && !Double.isNaN(t90)) r.riseS = t90 - t10;
        // Settled only if the last samples are inside the band.
        if (Math.abs(angle[n - 1] - to) <= band) r.settleS = lastOutside;
        double tEnd = t[n - 1], sum = 0;
        int cnt = 0;
        for (int i = 0; i < n; i++) if (t[i] >= tEnd - 0.2) { sum += to - angle[i]; cnt++; }
        r.finalErrDeg = cnt > 0 ? Math.toDegrees(sum / cnt) : 0;
        return r;
    }

    /**
     * PD gains for the plant  speed' = (K * power - speed) / tau  (a first-order motor lag on an
     * integrator): closed loop  tau s^2 + (1 + K Kd) s + K Kp = 0,  so for natural frequency w and
     * damping zeta:  Kp = tau w^2 / K,  Kd = (2 zeta w tau - 1) / K  (never below 0).
     * w is capped at 0.2 / loopS (the loop's own delay needs w * loop time well below 1).
     * Returns {kp, kd, w actually used}.
     */
    public static double[] designPd(double K, double tau, double wanted, double zeta, double loopS) {
        double w = Math.min(wanted, loopS > 1e-4 ? 0.2 / loopS : wanted);
        double kp = tau * w * w / K;
        double kd = Math.max(0, (2 * zeta * w * tau - 1) / K);
        return new double[]{kp, kd, w};
    }
}
