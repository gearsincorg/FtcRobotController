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
 * Collect and transfer: intake motor, gate servo and transfer motor, with four slot sensors.
 *
 *   - the intake motor pulls balls in at the mouth
 *   - the transfer moves the whole column toward the gate / shooter
 *   - the gate servo keeps balls out of the shooter until we shoot
 *   - four digital sensors, one per ball position (slot1 nearest the intake, slot4 at the gate)
 *
 * The ball count is simply the number of sensors that see a ball (briefly debounced).  No sensor
 * seeing a ball is "empty", all four is "full".  Nothing is inferred from a ball passing a sensor.
 *
 * During init (until START) the gate sits at GATE_INIT; after that the mode sets it.
 *
 * The mode is worked out every loop from what the driver is holding and the ball count, highest
 * priority first:
 *
 *   SHOOTING   shoot held and the shooter is up to speed.  Gate open, intake in,
 *              transfer feeds.  The only mode with the gate open.
 *   SPINUP     shoot held but the shooter isn't up to speed (yet, or again after a shot slowed it):
 *              intake and transfer stopped, gate closed.  While shoot is held (SHOOTING or SPINUP), clear and eject are ignored.
 *   CLEAR_ALL  clear held: intake out, transfer reversed, purges every ball.
 *   EJECTING   eject held: intake out, transfer holds the column.
 *   FULL       four balls: intake holds gently (INTAKE_HOLD_POWER), transfer at TRANSFER_PACKED_POWER.
 *   INTAKING   otherwise: intake in.  The transfer runs at TRANSFER_POWER until the balls are
 *              packed (stacked from slot4 down with no gaps, see isPacked()), then TRANSFER_PACKED_POWER so
 *              it doesn't stall against the closed gate.  A new ball at slot1 makes a gap and
 *              starts it again.
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
    private static final String[] SLOT_NAMES = { "slot1", "slot2", "slot3", "slot4" };

    // == Tuning (all placeholders) ===========================================
    // Intake motor
    private static final double INTAKE_POWER          = 1.0;   // in             (INTAKING, SHOOTING)
    private static final double INTAKE_HOLD_POWER     = 0.1;   // hold           (FULL)
    private static final double INTAKE_REJECT_POWER   =-0.3;   // out            (EJECTING)
    private static final double INTAKE_EJECT_POWER    =-1.0;   // out, full speed (CLEAR_ALL)
    // Transfer motor
    private static final double TRANSFER_POWER        = 1.0;   // up             (SHOOTING; INTAKING while not packed)
    private static final double TRANSFER_PACKED_POWER = 0.0;   // packed         (INTAKING while packed, FULL), balls stacked against the closed gate
    private static final double TRANSFER_HOLD_POWER   = 0.2;   // hold           (EJECTING), keeps the column in place
    private static final double TRANSFER_EJECT_POWER  =-1.0;   // down           (CLEAR_ALL)

    // TODO tune on the robot.
    public static final double GATE_CLOSED         = 0.35;
    public static final double GATE_OPEN           = 0.44;
    public static final double GATE_INIT           = 0.5;   // during init only (until START).  Placeholder.

    public  static final double COUNT_DEBOUNCE_MS  = 100;   // sensor pattern must hold this long
    private static final double VOLLEY_TIMEOUT_MS  = 2000;  // auto volley: stop this long after feeding first starts (spin-up dips included)
    private static final double SHOT_CLEAR_MS      = 300;   // auto volley: last ball leaves the flywheel

    public enum Mode { INTAKING, FULL, EJECTING, CLEAR_ALL, SPINUP, SHOOTING }

    private final OpMode          opMode;
    private final BooleanSupplier shooterReady;
    private DcMotorEx        intake;
    private DcMotorEx        transfer;
    private Servo            gate;
    private DigitalChannel[] slotSensors = new DigitalChannel[SLOT_NAMES.length];
    private SimulatedSlots   sim;

    // Sensors
    private boolean[] slots         = new boolean[SLOT_NAMES.length];   // debounced, slot1 first
    private boolean[] pendingSlots  = new boolean[SLOT_NAMES.length];   // raw, waiting to hold steady
    private String    pattern       = "";
    private int       ballCount     = 0;      // balls in the debounced pattern

    // Driver requests
    private boolean  shootHeld       = false;
    private boolean  clearHeld       = false;
    private boolean  ejectHeld       = false;
    private boolean  fedThisPress    = false;  // SHOOTING has been reached since shoot was pressed

    // State
    private Mode     mode          = Mode.INTAKING;

    // Outputs (also what the simulation reacts to)
    private double  intakePower    = 0;
    private double  transferPower  = 0;
    private double  gatePosition   = GATE_CLOSED;

    private final ElapsedTime countTimer = new ElapsedTime();   // time the raw pattern has been stable
    private final ElapsedTime volleyTimer = new ElapsedTime();  // time since this press first reached SHOOTING
    private final ElapsedTime emptyTimer = new ElapsedTime();   // time the count has read empty

    // == standard Methods ===================================
    /** shooterReady: true when the shooter can take a ball (normally ShooterSubsystem::isReady). */
    public IntakeSubsystem(OpMode myOpMode, BooleanSupplier shooterReady) {
        opMode = myOpMode;
        this.shooterReady = shooterReady;
    }

    public void init() {
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        transfer = opMode.hardwareMap.get(DcMotorEx.class, "transfer");
        gate   = opMode.hardwareMap.tryGet(Servo.class, "gate");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (SIMULATE_SENSORS) {
            sim = new SimulatedSlots(SIM_PRELOAD);
        } else {
            for (int i = 0; i < SLOT_NAMES.length; i++) {
                slotSensors[i] = opMode.hardwareMap.get(DigitalChannel.class, SLOT_NAMES[i]);
                slotSensors[i].setMode(DigitalChannel.Mode.INPUT);
            }
        }

        readSensors();
        slots = pendingSlots.clone();               // trust the first reading (preload)
        ballCount = countBalls(slots);

        // Motors off and gate at GATE_INIT until the first loop after START (the scheduler doesn't
        // run during init); from then on the mode decides, so the gate closes once intaking starts.
        set(0, 0, GATE_INIT);
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
                intakePower, transferPower, gatePosition);
        opMode.telemetry.addData("BALLS", "%d  [ %s]  %s%s", ballCount, pattern, mode,
                isPacked() ? "  PACKED" : "");
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
    public int      ballCount()   { return ballCount; }
    public boolean  isFull()      { return ballCount == slots.length; }
    public boolean  isEmpty()     { return ballCount == 0; }

    /**
     * The balls are stacked at the top with no gaps: from slot4 down it's balls, then only empty
     * slots (e.g. "- - X X").  Empty is not packed.
     */
    public boolean isPacked() {
        if (isEmpty()) return false;
        int i = slots.length - 1;
        while (i >= 0 && slots[i]) i--;     // the stack of balls under the gate
        while (i >= 0 && !slots[i]) i--;    // then nothing below it
        return i < 0;
    }
    /** True while shoot is requested (SPINUP or SHOOTING). */
    public boolean  isShooting()  { return shootHeld; }
    public String   slotPattern() { return pattern; }

    public double getGatePosition() { return gatePosition; }

    /** An auto volley is done: the column has read empty long enough, or we've fed too long. */
    private boolean volleyDone() {
        if (!fedThisPress) {
            return false;
        }
        return (isEmpty() && emptyTimer.milliseconds() > SHOT_CLEAR_MS)
                || volleyTimer.milliseconds() > VOLLEY_TIMEOUT_MS;
    }

    // == Sensors ==============================================================

    private void readSensors() {
        boolean[] raw;
        if (sim != null) {
            sim.update(intakePower, transferPower, isGateOpen());
            raw = sim.read();
        } else {
            raw = new boolean[SLOT_NAMES.length];
            for (int i = 0; i < raw.length; i++) {
                raw[i] = slotSensors[i].getState() == BALL_SEEN_STATE;
            }
        }

        StringBuilder sb = new StringBuilder();
        for (boolean ball : raw) {
            sb.append(ball ? "X " : "- ");
        }
        pattern = sb.toString();

        // Use the pattern once it has held steady, so a ball rolling between sensors doesn't
        // flicker the count or the packed state.
        if (!java.util.Arrays.equals(raw, pendingSlots)) {
            pendingSlots = raw;
            countTimer.reset();
        } else if (countTimer.milliseconds() > COUNT_DEBOUNCE_MS) {
            slots = raw;
            ballCount = countBalls(slots);
        }

        if (!isEmpty()) {
            emptyTimer.reset();
        }
    }

    private static int countBalls(boolean[] seen) {
        int count = 0;
        for (boolean ball : seen) {
            if (ball) count++;
        }
        return count;
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
                set(INTAKE_POWER, TRANSFER_POWER, GATE_OPEN);
                break;
            case SPINUP:
                set(0, 0, GATE_CLOSED);
                break;
            case CLEAR_ALL:
                set(INTAKE_EJECT_POWER, TRANSFER_EJECT_POWER, GATE_CLOSED);
                break;
            case EJECTING:
                set(INTAKE_REJECT_POWER, TRANSFER_HOLD_POWER, GATE_CLOSED);
                break;
            case FULL:
                set(INTAKE_HOLD_POWER, TRANSFER_PACKED_POWER, GATE_CLOSED);
                break;
            case INTAKING:
            default:
                // Move balls up until they're stacked under the gate, then ease off so the
                // transfer doesn't stall against the closed gate.
                set(INTAKE_POWER, isPacked() ? TRANSFER_PACKED_POWER : TRANSFER_POWER, GATE_CLOSED);
                break;
        }
    }

    private void set(double intakePower, double transferPower, double gatePosition) {
        this.intakePower  = intakePower;
        this.transferPower = transferPower;
        this.gatePosition = gatePosition;
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
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
