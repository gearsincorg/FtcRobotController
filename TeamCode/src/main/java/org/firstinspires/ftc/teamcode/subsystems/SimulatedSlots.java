package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Stand-in for the four slot sensors until they are installed.
 *
 * Models the ball column as positions measured in slots:
 *   0 = intake mouth, 1..4 = the slot sensors, 4 = the gate, past SHOT_POS = in the shooter.
 * Each loop, balls move with the powers the intake subsystem is commanding:
 *   - the intake roller moves balls near the mouth
 *   - the feeder moves the whole column
 *   - balls keep one slot of spacing, and stop at the gate while it is closed
 * Balls that pass the open gate are "shot"; balls reversed out of the mouth are "ejected".
 * While the intake runs, a new ball arrives at the mouth every PICKUP_MS (as if driving over
 * field balls), so auto can collect in simulation too.
 */
public class SimulatedSlots {

    private static final int    SLOTS              = 4;
    private static final double GATE_POS           = 4.0;
    private static final double SHOT_POS           = 4.6;   // past here the ball is in the shooter
    private static final double EJECT_POS          = -0.5;  // below here the ball is out the mouth
    private static final double SPACING            = 1.0;   // one ball per slot
    private static final double SENSOR_HALF_WIDTH  = 0.4;
    private static final double INTAKE_ZONE        = 1.0;   // the intake roller reaches balls below this
    private static final double INTAKE_SPEED       = 4.0;   // slots per second at full power
    private static final double FEED_SPEED         = 4.0;   // slots per second at full power
    private static final double PICKUP_MS          = 500;
    private static final double MAX_DT             = 0.1;

    // Ball positions, front (nearest the shooter) first.
    private final List<Double> balls = new ArrayList<>();
    private final ElapsedTime loopTimer   = new ElapsedTime();
    private final ElapsedTime pickupTimer = new ElapsedTime();
    private int shot    = 0;
    private int ejected = 0;

    public SimulatedSlots(int preload) {
        for (int i = 0; i < Math.min(preload, SLOTS); i++) {
            balls.add(GATE_POS - i * SPACING);
        }
    }

    /** Advance the model by the time since the last call. */
    public void update(double intakePower, double feederPower, boolean gateOpen) {
        double dt = Math.min(loopTimer.seconds(), MAX_DT);
        loopTimer.reset();

        // Pick up a field ball now and then while the intake is pulling in.
        if (intakePower > 0.1) {
            if (pickupTimer.milliseconds() > PICKUP_MS && mouthClear()) {
                balls.add(0.0);
                pickupTimer.reset();
            }
        } else {
            pickupTimer.reset();
        }

        // Move every ball, then enforce the gate and spacing from the front back.
        // A ball already past the gate is in the flywheel and keeps going.
        double limit = Double.MAX_VALUE;
        for (int i = 0; i < balls.size(); i++) {
            double p = balls.get(i);
            if (p > GATE_POS + 0.05) {
                balls.set(i, p + FEED_SPEED * dt);
                limit = p - SPACING;
                continue;
            }
            if (!gateOpen) {
                limit = Math.min(limit, GATE_POS);
            }
            double v = feederPower * FEED_SPEED;
            if (p < INTAKE_ZONE) {
                v += intakePower * INTAKE_SPEED;
            }
            p = Math.min(p + v * dt, limit);
            balls.set(i, p);
            limit = p - SPACING;
        }

        // Remove balls that left the robot.
        for (int i = balls.size() - 1; i >= 0; i--) {
            double p = balls.get(i);
            if (p > SHOT_POS) {
                balls.remove(i);
                shot++;
            } else if (p < EJECT_POS) {
                balls.remove(i);
                ejected++;
            }
        }
    }

    /** Sensor readings, slot1 first. */
    public boolean[] read() {
        boolean[] seen = new boolean[SLOTS];
        for (double p : balls) {
            for (int k = 1; k <= SLOTS; k++) {
                if (Math.abs(p - k) < SENSOR_HALF_WIDTH) {
                    seen[k - 1] = true;
                }
            }
        }
        return seen;
    }

    /** Put a ball at the intake mouth (if there is room). */
    public void addBall() {
        if (mouthClear()) {
            balls.add(0.0);
        }
    }

    /** Take away the ball nearest the intake. */
    public void removeBall() {
        if (!balls.isEmpty()) {
            balls.remove(balls.size() - 1);
        }
    }

    public int ballsInRobot() { return balls.size(); }
    public int shotCount()    { return shot; }
    public int ejectedCount() { return ejected; }

    private boolean mouthClear() {
        return balls.isEmpty() || balls.get(balls.size() - 1) >= SPACING;
    }
}
