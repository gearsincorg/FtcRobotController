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

import java.util.function.BooleanSupplier;

/**
 * Collect and transfer: intake motor, gate servo and feeder motor, with four slot sensors.
 *
 *   - the intake motor pulls balls in at the mouth
 *   - the feeder moves the whole column toward the gate / shooter
 *   - the gate servo keeps balls out of the shooter until we shoot
 *   - four digital sensors, one per ball position (slot1 nearest the intake, slot4 at the gate)
 *
 * The ball count is simply the number of sensors that see a ball (briefly debounced).  No sensor
 * seeing a ball is "empty", all four is "full".  Nothing is inferred from a ball passing a sensor.
 *
 * What the motors are doing (Mode) is separate from the ball count:
 *
 *   IDLE       everything off, gate closed
 *   INTAKING   intake in, feeder slowly packs balls against the closed gate -> REJECTING when full
 *   REJECTING  intake reversed briefly to spit out a 5th ball, feeder holds the four
 *   SHOOTING   gate open, fires the whole load in one volley, steps WAIT_READY -> FEED -> CLEARING:
 *                WAIT_READY  wait for the shooter to be ready
 *                FEED        run the feeder until the column reads empty (or VOLLEY_TIMEOUT_MS)
 *                CLEARING    keep feeding SHOT_CLEAR_MS so the last ball leaves the flywheel
 *              Auto fires a volley with shootAllCommand(); teleop fires while a button is held
 *              with shootWhileHeldCommand().
 *   EJECTING   intake and feeder reversed: purges every ball out of the intake
 *   MANUAL     powers and gate set directly (test opmode, gamepad 2 fallback)
 *
 * Until the sensors are in use, SIMULATE_SENSORS swaps them for SimulatedSlots.
 */
public class IntakeSubsystem extends SubsystemBase {

    // == Sensors =============================================================
    // true: the ball count comes from SimulatedSlots instead of the slot sensors.
    // TODO set false once the slot sensors are in use.
    public static final boolean SIMULATE_SENSORS = true;
    private static final int    SIM_PRELOAD      = 4;

    // Sensor reading when a ball is in the slot (most digital sensors are active-low).
    private static final boolean BALL_SEEN_STATE = false;
    private static final String[] SLOT_NAMES = { "slot1", "slot2", "slot3", "slot4" };

    // == Tuning (all placeholders) ===========================================
    private static final double INTAKE_POWER       = 1.0;
    private static final double PACK_POWER         = 1.0;   // feeder while intaking, against the closed gate
    private static final double HOLD_POWER         = 0.2;   // feeder while rejecting, keeps the four in place
    private static final double FEED_POWER         = 1.0;
    private static final double REJECT_POWER       = -1.0;
    private static final double EJECT_POWER        = -1.0;

    // TODO tune with Util/IntakeTestOpMode.
    public static final double GATE_CLOSED         = 0.0;
    public static final double GATE_OPEN           = 0.5;

    private static final double COUNT_DEBOUNCE_MS  = 100;   // sensor count must hold this long
    private static final double REJECT_MS          = 400;
    private static final double VOLLEY_TIMEOUT_MS  = 3000;  // stop feeding even if a sensor still sees a ball
    private static final double SHOT_CLEAR_MS      = 300;   // last ball leaves the flywheel

    public enum Mode     { IDLE, INTAKING, REJECTING, SHOOTING, EJECTING, MANUAL }
    public enum ShotStep { WAIT_READY, FEED, CLEARING }

    private final OpMode          opMode;
    private final BooleanSupplier shooterReady;
    private DcMotorEx        intake;
    private DcMotorEx        feeder;
    private Servo            gate;
    private DigitalChannel[] slotSensors = new DigitalChannel[SLOT_NAMES.length];
    private SimulatedSlots   sim;

    // Sensors
    private boolean[] slots         = new boolean[SLOT_NAMES.length];
    private String    pattern       = "";
    private int       ballCount     = 0;      // debounced
    private int       pendingCount  = 0;

    // State
    private Mode     mode          = Mode.IDLE;
    private ShotStep shotStep      = ShotStep.WAIT_READY;
    private boolean  intakeHeld    = false;

    // Outputs (also what the simulation reacts to)
    private double  intakePower    = 0;
    private double  feederPower    = 0;
    private double  gatePosition   = GATE_CLOSED;
    private double  manualIntake   = 0;
    private double  manualFeeder   = 0;
    private double  manualGate     = GATE_CLOSED;

    private final ElapsedTime countTimer    = new ElapsedTime();   // time the raw count has been stable
    private final ElapsedTime modeTimer     = new ElapsedTime();   // time in the current mode / step

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

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (SIMULATE_SENSORS) {
            sim = new SimulatedSlots(SIM_PRELOAD);
        } else {
            for (int i = 0; i < SLOT_NAMES.length; i++) {
                slotSensors[i] = opMode.hardwareMap.get(DigitalChannel.class, SLOT_NAMES[i]);
                slotSensors[i].setMode(DigitalChannel.Mode.INPUT);
            }
        }

        readSensors();
        ballCount = pendingCount = rawCount();      // trust the first reading (preload)
        setMode(Mode.IDLE);
        applyOutputs();
    }

    @Override
    public void periodic() {
        readSensors();
        updateState();
        applyOutputs();
        showTelemetry();
    }

    void showTelemetry() {
        if (sim != null) {
            opMode.telemetry.addLine("SIMULATED BALL SENSORS");
        }
        opMode.telemetry.addData("In:Feed:Gate", "%4.2f, %4.2f, %4.2f",
                intakePower, feederPower, gatePosition);
        opMode.telemetry.addData("BALLS", "%d  [ %s]  %s",
                ballCount, pattern, modeName());
    }

    /** Read the sensors without running anything else (e.g. to show the preload during init). */
    public void refreshSensors() {
        readSensors();
    }

    // == Requests =============================================================

    /** Start collecting (stays on while held; stops and rejects by itself when full). */
    public void startIntake() {
        intakeHeld = true;
        if (mode != Mode.SHOOTING && mode != Mode.EJECTING && !isFull()) {
            setMode(Mode.INTAKING);
        }
    }

    public void stopIntake() {
        intakeHeld = false;
        if (mode == Mode.INTAKING) {
            setMode(Mode.IDLE);
        }
    }

    /** Fire every ball in one volley (stops when the column reads empty). */
    public void shootAll() {
        if (isEmpty()) {
            return;
        }
        setMode(Mode.SHOOTING);
        setShotStep(ShotStep.WAIT_READY);
    }

    public void cancelShoot() {
        if (mode == Mode.SHOOTING) {
            setMode(Mode.IDLE);
        }
    }

    /** Purge every ball out of the intake (overrides everything else). */
    public void startEject() {
        setMode(Mode.EJECTING);
    }

    public void stopEject() {
        if (mode == Mode.EJECTING) {
            setMode(Mode.IDLE);
        }
    }

    /** Drive the hardware directly.  Sensors and counts keep updating. */
    public void setManual(double intakePower, double feederPower, double gatePosition) {
        manualIntake = intakePower;
        manualFeeder = feederPower;
        manualGate   = gatePosition;
        if (mode != Mode.MANUAL) {
            setMode(Mode.MANUAL);
        }
    }

    public void clearManual() {
        if (mode == Mode.MANUAL) {
            setMode(Mode.IDLE);
        }
    }

    // Simulation helpers (do nothing with real sensors).
    public void simAddBall()     { if (sim != null) sim.addBall(); }
    public void simRemoveBall()  { if (sim != null) sim.removeBall(); }
    public boolean isSimulated() { return sim != null; }

    // == Queries ==============================================================

    public Mode     getMode()     { return mode; }
    public ShotStep getShotStep() { return shotStep; }
    public int      ballCount()   { return ballCount; }
    public boolean  isFull()      { return ballCount == slots.length; }
    public boolean  isEmpty()     { return ballCount == 0; }
    public boolean  isShooting()  { return mode == Mode.SHOOTING; }
    public String   slotPattern() { return pattern; }

    public boolean isWaitingForShooter() {
        return mode == Mode.SHOOTING && shotStep == ShotStep.WAIT_READY;
    }

    public double getGatePosition() { return gatePosition; }

    // == Sensors ==============================================================

    private void readSensors() {
        boolean[] raw;
        if (sim != null) {
            sim.update(intakePower, feederPower, isGateOpen());
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
        slots = raw;

        // The count is the number of sensors seeing a ball, once it has held steady.
        int count = rawCount();
        if (count != pendingCount) {
            pendingCount = count;
            countTimer.reset();
        } else if (countTimer.milliseconds() > COUNT_DEBOUNCE_MS) {
            ballCount = count;
        }
    }

    private int rawCount() {
        int count = 0;
        for (boolean ball : slots) {
            if (ball) count++;
        }
        return count;
    }

    private boolean isGateOpen() {
        return Math.abs(gatePosition - GATE_OPEN) < Math.abs(gatePosition - GATE_CLOSED);
    }

    // == State machine =========================================================

    private void updateState() {
        switch (mode) {
            case IDLE:
                if (intakeHeld && !isFull()) {
                    setMode(Mode.INTAKING);
                }
                break;

            case INTAKING:
                if (isFull()) {
                    setMode(Mode.REJECTING);
                }
                break;

            case REJECTING:
                if (modeTimer.milliseconds() > REJECT_MS) {
                    setMode(Mode.IDLE);
                }
                break;

            case SHOOTING:
                updateShooting();
                break;

            default:
                break;
        }
    }

    /** One volley: wait for the shooter once, then feed until the column reads empty. */
    private void updateShooting() {
        switch (shotStep) {
            case WAIT_READY:
                if (shooterReady.getAsBoolean()) {
                    setShotStep(ShotStep.FEED);
                }
                break;

            case FEED:
                if (isEmpty() || modeTimer.milliseconds() > VOLLEY_TIMEOUT_MS) {
                    setShotStep(ShotStep.CLEARING);
                }
                break;

            case CLEARING:
                if (modeTimer.milliseconds() > SHOT_CLEAR_MS) {
                    setMode(Mode.IDLE);
                }
                break;
        }
    }

    private void setMode(Mode newMode) {
        mode = newMode;
        modeTimer.reset();
    }

    private void setShotStep(ShotStep step) {
        shotStep = step;
        modeTimer.reset();
    }

    private void applyOutputs() {
        switch (mode) {
            case INTAKING:
                set(INTAKE_POWER, PACK_POWER, GATE_CLOSED);
                break;
            case REJECTING:
                set(REJECT_POWER, HOLD_POWER, GATE_CLOSED);
                break;
            case SHOOTING:
                if (shotStep != ShotStep.WAIT_READY) {
                    // The feeder moves the whole column; leave the intake off so we don't
                    // collect more while shooting.
                    set(0, FEED_POWER, GATE_OPEN);
                } else {
                    set(0, 0, GATE_OPEN);
                }
                break;
            case EJECTING:
                set(EJECT_POWER, EJECT_POWER, GATE_CLOSED);
                break;
            case MANUAL:
                set(manualIntake, manualFeeder, manualGate);
                break;
            default:
                set(0, 0, GATE_CLOSED);
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

    private String modeName() {
        return mode == Mode.SHOOTING ? "SHOOTING/" + shotStep : mode.toString();
    }

    // == Command methods  ==========================================

    /** Collect while scheduled (e.g. while a button is held). */
    public Command intakeCommand() {
        return new StartEndCommand(this::startIntake, this::stopIntake, this);
    }

    /** Collect until full (for auto; add a timeout). */
    public Command intakeUntilFullCommand() {
        return new FunctionalCommand(
                this::startIntake,
                () -> {},
                interrupted -> stopIntake(),
                this::isFull,
                this);
    }

    /** Fire every ball in one volley (auto); finishes when the last one has left.  Cancels if interrupted. */
    public Command shootAllCommand() {
        return new FunctionalCommand(
                this::shootAll,
                () -> {},
                interrupted -> { if (interrupted) cancelShoot(); },
                () -> mode != Mode.SHOOTING,
                this);
    }

    /** Fire while scheduled (e.g. while a button is held); stops feeding when released. */
    public Command shootWhileHeldCommand() {
        return new StartEndCommand(this::shootAll, this::cancelShoot, this);
    }

    /** Purge everything out of the intake while scheduled. */
    public Command ejectCommand() {
        return new StartEndCommand(this::startEject, this::stopEject, this);
    }
}
