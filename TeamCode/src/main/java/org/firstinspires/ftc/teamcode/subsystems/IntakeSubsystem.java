package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;

/**
 * Collect and transfer: intake motor, gate servo and feeder (transfer) motor, with three sensors.
 *
 *   - the intake motor pulls balls in at the mouth
 *   - the feeder moves the whole column toward the gate / shooter
 *   - the gate servo keeps balls out of the shooter until we shoot
 *   - two digital sensors at the very front of the intake (frontA, frontB) and one at the top (top)
 *
 * There is no ball count, just three states:
 *   FULL       either front sensor has seen something continuously for FULL_MS (balls backed up to the mouth)
 *   EMPTY      no sensor has seen anything for EMPTY_MS
 *   otherwise  we have balls
 *
 * The mode is worked out every loop from what the driver is holding and the ball count, highest
 * priority first:
 *
 *   SHOOTING   shoot held and the shooter is up to speed.  Gate open, intake in,
 *              feeder feeds.  The only mode with the gate open.
 *   SPINUP     shoot held but the shooter isn't up to speed (yet, or again after a shot slowed it):
 *              intake and feeder stopped, gate closed.  While shoot is held (SHOOTING or SPINUP), clear and eject are ignored.
 *   CLEAR_ALL  clear held: intake out, feeder reversed, purges every ball.
 *   EJECTING   eject held: intake out, feeder holds the column.
 *   FULL       front sensors blocked for FULL_MS: intake out (spits extra balls), feeder packs the column.
 *   INTAKING   otherwise: intake in, feeder feeds the column against the closed gate.
 *
 * Until the sensors are in use, SIMULATE_SENSORS swaps them for SimulatedSlots.
 */
public class IntakeSubsystem extends SubsystemBase {

    // == Sensors =============================================================
    // true: the ball count comes from SimulatedSlots instead of the slot sensors.
    // TODO set false once the slot sensors are in use.
    public static final boolean SIMULATE_SENSORS = false;
    private static final int    SIM_PRELOAD      = 4;

    // Sensor reading when a ball is in the slot (most digital sensors are active-low).
    private static final boolean BALL_SEEN_STATE = false;
    private static final String[] SENSOR_NAMES = { "frontA", "frontB", "top" };
    private static final int      FRONT_A = 0, FRONT_B = 1, TOP = 2;

    // == Tuning (all placeholders) ===========================================
    private static final double INTAKE_POWER       = 1.0;   // intake in   (INTAKING, SHOOTING)
    private static final double REJECT_POWER       =-0.3;   // intake out  (FULL, EJECTING, CLEAR_ALL)
    private static final double FEED_POWER         = 1.0;   // feeder      (INTAKING, SHOOTING)
    private static final double PACK_POWER         = 0.0;   // feeder      (FULL), against the closed gate
    private static final double HOLD_POWER         = 0.2;   // feeder      (EJECTING), keeps the column in place
    private static final double EJECT_POWER        = -1.0;  // feeder      (CLEAR_ALL)

    // TODO tune on the robot.
    public static final double GATE_CLOSED         = 0.0;
    public static final double GATE_OPEN           = 0.5;

    private static final boolean TOP_ONLY_FULL     = true;  // TEMPORARY test: full whenever the top sensor sees something (ignores front sensors and FULL_MS)
    private static final double FULL_MS           = 500; // a front sensor must see something this long to read full
    private static final double SIM_FULL_MS        = 1000;  // same, for the simulated sensors
    private static final double EMPTY_MS           = 500;   // no sensor sees anything for this long to read empty
    private static final double VOLLEY_TIMEOUT_MS  = 2000;  // auto volley: stop this long after feeding first starts (spin-up dips included)

    public enum Mode { INTAKING, FULL, EJECTING, CLEAR_ALL, SPINUP, SHOOTING }

    private final OpMode          opMode;
    private final BooleanSupplier shooterReady;
    private DcMotorEx        intake;
    private DcMotorEx        feeder;
    private Servo            gate;
    private DigitalChannel[] sensors = new DigitalChannel[SENSOR_NAMES.length];
    private SimulatedSlots   sim;

    // Sensors
    private String    pattern       = "";

    // Driver requests
    private boolean  shootHeld       = false;
    private boolean  clearHeld       = false;
    private boolean  ejectHeld       = false;
    private boolean  fedThisPress    = false;  // SHOOTING has been reached since shoot was pressed

    // State
    private Mode     mode          = Mode.INTAKING;

    // Outputs (also what the simulation reacts to)
    private double  intakePower    = 0;
    private double  feederPower    = 0;
    private double  gatePosition   = GATE_CLOSED;

    private final ElapsedTime frontTimer = new ElapsedTime();   // time a front sensor has continuously seen something
    private boolean topNow  = false;                            // top sensor is seeing something right now
    private boolean topSeen = false;                           // top sensor has seen something since the intake was last empty
    private final ElapsedTime volleyTimer = new ElapsedTime();  // time since this press first reached SHOOTING
    private final ElapsedTime emptyTimer = new ElapsedTime();   // time no sensor has seen anything

    // == standard Methods ===================================
    /** shooterReady: true when the shooter can take a ball (normally ShooterSubsystem::isReady). */
    public IntakeSubsystem(OpMode myOpMode, BooleanSupplier shooterReady) {
        opMode = myOpMode;
        this.shooterReady = shooterReady;
    }

    public void init() {
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        feeder = opMode.hardwareMap.get(DcMotorEx.class, "feeder");
        gate   = opMode.hardwareMap.tryGet(Servo.class, "gate");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (SIMULATE_SENSORS) {
            sim = new SimulatedSlots(SIM_PRELOAD);
        } else {
            for (int i = 0; i < SENSOR_NAMES.length; i++) {
                sensors[i] = opMode.hardwareMap.get(DigitalChannel.class, SENSOR_NAMES[i]);
                sensors[i].setMode(DigitalChannel.Mode.INPUT);
            }
        }

        readSensors();

        // Everything off until the first loop after START (the scheduler doesn't run during init).
        set(0, 0, GATE_CLOSED);
    }

    @Override
    public void periodic() {
        readSensors();
        updateMode();
        applyOutputs();
        showTelemetry();
    }

    void showTelemetry() {
        if (sim != null) {
            opMode.telemetry.addLine("SIMULATED BALL SENSORS");
        }
        opMode.telemetry.addData("In:Feed:Gate", "%4.2f, %4.2f, %4.2f",
                intakePower, feederPower, gatePosition);
        opMode.telemetry.addData("BALLS", "%s  [ %s]  %s", stateName(), pattern, mode);
    }

    /** Read the sensors without running anything else (e.g. to show the preload during init). */
    public void refreshSensors() {
        readSensors();
    }

    // == Requests =============================================================

    public void startShoot() { shootHeld = true; }

    public void stopShoot() {
        shootHeld = false;
        fedThisPress = false;
    }

    public void startClear() { clearHeld = true; }
    public void stopClear()  { clearHeld = false; }

    public void startEject() { ejectHeld = true; }
    public void stopEject()  { ejectHeld = false; }

    // Simulation helpers (do nothing with real sensors).
    public void simAddBall()     { if (sim != null) sim.addBall(); }
    public void simRemoveBall()  { if (sim != null) sim.removeBall(); }
    public boolean isSimulated() { return sim != null; }

    // == Queries ==============================================================

    public Mode     getMode()     { return mode; }
    /** A front sensor has seen something for FULL_MS, and the top sensor has seen something since last empty. */
    public boolean  isFull()      { return TOP_ONLY_FULL ? topNow : topSeen &&frontTimer.milliseconds() > (sim != null ? SIM_FULL_MS : FULL_MS); }
    /** No sensor has seen anything for EMPTY_MS. */
    public boolean  isEmpty()     { return emptyTimer.milliseconds() > EMPTY_MS; }
    public boolean  hasBalls()    { return !isEmpty(); }
    public String   stateName()   { return isFull() ? "FULL" : isEmpty() ? "EMPTY" : "HAS BALLS"; }
    /** True while shoot is requested (SPINUP or SHOOTING). */
    public boolean  isShooting()  { return shootHeld; }
    public String   slotPattern() { return pattern; }

    public double getGatePosition() { return gatePosition; }

    /** An auto volley is done: the intake has read empty, or we've fed too long. */
    private boolean volleyDone() {
        if (!fedThisPress) {
            return false;
        }
        return isEmpty() || volleyTimer.milliseconds() > VOLLEY_TIMEOUT_MS;
    }

    // == Sensors ==============================================================

    private void readSensors() {
        boolean[] raw;
        if (sim != null) {
            sim.update(intakePower, feederPower, isGateOpen());
            raw = sim.read();
        } else {
            raw = new boolean[SENSOR_NAMES.length];
            for (int i = 0; i < raw.length; i++) {
                raw[i] = sensors[i].getState() == BALL_SEEN_STATE;
            }
        }

        StringBuilder sb = new StringBuilder();
        for (boolean ball : raw) {
            sb.append(ball ? "X " : "- ");
        }
        pattern = sb.toString();

        // Each timer runs while its condition is false, so it reads how long the condition has held.
        if (!(raw[FRONT_A] || raw[FRONT_B])) {
            frontTimer.reset();
        }
        if (raw[FRONT_A] || raw[FRONT_B] || raw[TOP]) {
            emptyTimer.reset();
        }
        // Full also needs the top sensor to have seen something at some point since last empty.
        topNow = raw[TOP];
        if (raw[TOP]) {
            topSeen = true;
        } else if (isEmpty()) {
            topSeen = false;
        }
    }

    private boolean isGateOpen() {
        return Math.abs(gatePosition - GATE_OPEN) < Math.abs(gatePosition - GATE_CLOSED);
    }

    // == Mode =================================================================

    /** Pick the mode from the driver's requests and the ball count, highest priority first. */
    private void updateMode() {
        Mode newMode;
        if (shootHeld) {
            // Shoot overrides clear and eject.  Feed only while the shooter is up to speed.
            newMode = shooterReady.getAsBoolean() ? Mode.SHOOTING : Mode.SPINUP;
        } else if (clearHeld) {
            newMode = Mode.CLEAR_ALL;
        } else if (ejectHeld) {
            newMode = Mode.EJECTING;
        } else if (isFull()) {
            newMode = Mode.FULL;
        } else {
            newMode = Mode.INTAKING;
        }

        if (newMode == Mode.SHOOTING && !fedThisPress) {
            fedThisPress = true;
            volleyTimer.reset();
        }
        mode = newMode;
    }

    private void applyOutputs() {
        switch (mode) {
            case SHOOTING:
                set(INTAKE_POWER, FEED_POWER, GATE_OPEN);
                break;
            case SPINUP:
                set(0, 0, GATE_CLOSED);
                break;
            case CLEAR_ALL:
                set(EJECT_POWER, EJECT_POWER, GATE_CLOSED);
                break;
            case EJECTING:
                set(REJECT_POWER, HOLD_POWER, GATE_CLOSED);
                break;
            case FULL:
                set(REJECT_POWER, PACK_POWER, GATE_CLOSED);
                break;
            case INTAKING:
            default:
                set(INTAKE_POWER, FEED_POWER, GATE_CLOSED);
                break;
        }
    }

    private void set(double intakePower, double feederPower, double gatePosition) {
        this.intakePower  = intakePower;
        this.feederPower  = feederPower;
        this.gatePosition = gatePosition;
        intake.setPower(intakePower);
        feeder.setPower(feederPower);
        if (gate != null) gate.setPosition(gatePosition);
    }

    // == Command methods  ==========================================
    // These don't require the subsystem, so holding shoot and clear together doesn't cancel one
    // of them; updateMode() picks the winner.

    /** Wait until full (intaking is always on).  For auto; add a timeout. */
    public Command intakeUntilFullCommand() {
        return new WaitUntilCommand(this::isFull);
    }

    /** Fire every ball in one volley (auto); finishes when the last one has left. */
    public Command shootAllCommand() {
        return new FunctionalCommand(
                this::startShoot,
                () -> {},
                interrupted -> stopShoot(),
                this::volleyDone);
    }

    /** Fire while scheduled (e.g. while a button is held). */
    public Command shootWhileHeldCommand() {
        return new StartEndCommand(this::startShoot, this::stopShoot);
    }

    /** Intake out while scheduled; the transfer holds its balls. */
    public Command ejectCommand() {
        return new StartEndCommand(this::startEject, this::stopEject);
    }

    /** Purge every ball (intake and transfer reversed) while scheduled. */
    public Command clearAllCommand() {
        return new StartEndCommand(this::startClear, this::stopClear);
    }
}
