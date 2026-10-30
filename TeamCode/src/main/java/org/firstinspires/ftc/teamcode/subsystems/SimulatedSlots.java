package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Stand-in for the four slot sensors until they are installed.
 *
 * Just a ball count (0..4).  Balls sit packed against the gate, so a count of n lights the n
 * sensors nearest the gate (slot4, slot3, ...), like the real column would.  The count changes
 * with what the intake subsystem is commanding:
 *   - intake running in: a ball arrives every PICKUP_MS, until the column is full
 *   - feeder pushing through the open gate: a ball is shot every SHOT_MS
 *   - intake and feeder both reversed: a ball is ejected every EJECT_MS
 */
public class SimulatedSlots {

    private static final int    SLOTS      = 4;
    private static final double PICKUP_MS  = 100;
    private static final double SHOT_MS    = 250;
    private static final double EJECT_MS   = 300;
    private static final double MAX_DT     = 0.1;

    private int count;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double pickupMs = 0;
    private double shotMs   = 0;
    private double ejectMs  = 0;

    public SimulatedSlots(int preload) {
        count = Math.max(0, Math.min(preload, SLOTS));
    }

    /** Advance the model by the time since the last call. */
    public void update(double intakePower, double feederPower, boolean gateOpen) {
        double dtMs = Math.min(loopTimer.seconds(), MAX_DT) * 1000;
        loopTimer.reset();

        pickupMs = (intakePower > 0.1 && count < SLOTS) ? pickupMs + dtMs : 0;
        if (pickupMs >= PICKUP_MS) {
            count++;
            pickupMs = 0;
        }

        shotMs = (gateOpen && feederPower > 0.1 && count > 0) ? shotMs + dtMs : 0;
        if (shotMs >= SHOT_MS) {
            count--;
            shotMs = 0;
        }

        ejectMs = (intakePower < -0.1 && feederPower < -0.1 && count > 0) ? ejectMs + dtMs : 0;
        if (ejectMs >= EJECT_MS) {
            count--;
            ejectMs = 0;
        }
    }

    /** Sensor readings, slot1 first.  Balls are packed against the gate (slot4). */
    public boolean[] read() {
        boolean[] seen = new boolean[SLOTS];
        for (int i = 0; i < SLOTS; i++) {
            seen[SLOTS - 1 - i] = i < count;
        }
        return seen;
    }

    public void addBall() {
        if (count < SLOTS) count++;
    }

    public void removeBall() {
        if (count > 0) count--;
    }
}
