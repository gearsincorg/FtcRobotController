package org.firstinspires.ftc.teamcode.Subsystems;

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
 *   - four digital sensors, one per slot (slot1 nearest the intake, slot4 at the gate)
 *
 * What the motors are doing (Mode) is kept separate from how many balls we hold (ballCount):
 *
 *   IDLE       everything off, gate closed
 *   INTAKING   intake in, feeder slowly packs balls against the closed gate
 *              -> REJECTING when full, -> UNJAMMING if the column stops moving with a gap
 *   REJECTING  intake reversed briefly to spit out a 5th ball, feeder holds the four
 *   SHOOTING   gate open, steps WAIT_READY -> FEED -> CLEARING:
 *                WAIT_READY  wait for the shooter to be ready (and MIN_SHOT_GAP_MS after a shot)
 *                FEED        run the feeder until a ball leaves slot4 (one shot).  Paced fire goes
 *                            back to WAIT_READY after each ball; RAPID_FIRE keeps feeding.
 *                CLEARING    wait SHOT_CLEAR_MS for the last ball to leave the flywheel
 *   EJECTING   intake and feeder reversed: purges every ball out of the intake
 *   UNJAMMING  short reverse pulse, then back to INTAKING
 *   MANUAL     powers and gate set directly (test opmode, gamepad 2 fallback)
 *
 * A shot is counted when slot4 goes from "ball" to "no ball" while the gate is open and the
 * feeder is pushing.  "All shot" = the count is zero and SHOT_CLEAR_MS has passed.
 *
 * Until the sensors are installed, SIMULATE_SENSORS swaps them for SimulatedSlots.
 */
public class IntakeSubsystem extends SubsystemBase {

    // == Sensors =============================================================
    // TODO set false once the slot sensors are installed.
    public static final boolean SIMULATE_SENSORS = true;
    private static final int    SIM_PRELOAD      = 3;

    // Sensor reading when a ball is in the slot (most digital sensors are active-low).
    private static final boolean BALL_SEEN_STATE = false;
    private static final String[] SLOT_NAMES = { "slot1", "slot2", "slot3", "slot4" };

    // == Firing ==============================================================
    // true: feed continuously once ready.  false: wait for the shooter to be ready before every ball.
    public static final boolean RAPID_FIRE = false;

    // == Tuning (all placeholders) ===========================================
    private static final double INTAKE_POWER       = 1.0;
    private static final double PACK_POWER         = 0.3;   // feeder while intaking, against the closed gate
    private static final double HOLD_POWER         = 0.2;   // feeder while rejecting, keeps the four in place
    private static final double FEED_POWER         = 1.0;
    private static final double REJECT_POWER       = -1.0;
    private static final double EJECT_POWER        = -1.0;
    private static final double UNJAM_POWER        = -0.6;

    // TODO tune with Util/IntakeTestOpMode.
    public static final double GATE_CLOSED         = 0.0;
    public static final double GATE_OPEN           = 0.5;

    private static final double COUNT_DEBOUNCE_MS     = 100;
    private static final double REJECT_MS             = 400;
    private static final double JAM_MS                = 1000;  // column has a gap and isn't moving
    private static final double UNJAM_MS              = 250;
    private static final int    MAX_UNJAM_TRIES       = 3;
    private static final double MIN_SHOT_GAP_MS       = 150;   // let the flywheel speed reading catch up
    private static final double FEED_BALL_TIMEOUT_MS  = 1000;  // feeding without a shot = jam
    private static final double SHOT_CLEAR_MS         = 300;   // last ball leaves the flywheel

    public enum Mode     { IDLE, INTAKING, REJECTING, SHOOTING, EJECTING, UNJAMMING, MANUAL }
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
    private boolean   lastSlot4     = false;
    private boolean   shotThisFeed  = false;

    // State
    private Mode     mode          = Mode.IDLE;
    private ShotStep shotStep      = ShotStep.WAIT_READY;
    private boolean  shootOneOnly  = false;
    private boolean  intakeHeld    = false;
    private boolean  jammed        = false;
    private int      unjamTries    = 0;
    private int      shotsFired    = 0;

    // Outputs (also what the simulation reacts to)
    private double  intakePower    = 0;
    private double  feederPower    = 0;
    private double  gatePosition   = GATE_CLOSED;
    private double  manualIntake   = 0;
    private double  manualFeeder   = 0;
    private double  manualGate     = GATE_CLOSED;

    private final ElapsedTime countTimer    = new ElapsedTime();   // time the raw count has been stable
    private final ElapsedTime modeTimer     = new ElapsedTime();   // time in the current mode / step
    private final ElapsedTime lastShotTimer = new ElapsedTime();
    private final ElapsedTime patternTimer  = new ElapsedTime();   // time the slot pattern has been unchanged

    // == standard Methods ===================================
    /** shooterReady: true when the shooter can take a ball (normally ShooterSubsystem::isReady). */
    public IntakeSubsystem(OpMode myOpMode, BooleanSupplier shooterReady) {
        opMode = myOpMode;
        this.shooterReady = shooterReady;
    }

    public void init() {
        // Any of these missing from the robot configuration is simulated: the state machine and
        // SimulatedSlots still run on the commanded powers, the hardware call is just skipped.
        intake = opMode.hardwareMap.tryGet(DcMotorEx.class, "intake");
        feeder = opMode.hardwareMap.tryGet(DcMotorEx.class, "feeder");
        gate   = opMode.hardwareMap.tryGet(Servo.class, "gate");

        if (intake != null) {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (feeder != null) {
            feeder.setDirection(DcMotorSimple.Direction.FORWARD);
            feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

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
        lastShotTimer.reset();
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
            opMode.telemetry.addLine("!!! INTAKE: SIMULATED SENSORS !!!");
        }
        if (intake == null || feeder == null || gate == null) {
            opMode.telemetry.addLine("!!! INTAKE: SIMULATED "
                    + (intake == null ? "intake " : "")
                    + (feeder == null ? "feeder " : "")
                    + (gate == null ? "gate " : "")
                    + "(not in config) !!!");
        }
        opMode.telemetry.addData("Intake : feeder : gate", "%4.2f : %4.2f : %4.2f",
                intakePower, feederPower, gatePosition);
        opMode.telemetry.addData("BALLS", "%d  [ %s]  %s  shots %d",
                ballCount, pattern, modeName(), shotsFired);
        if (jammed) {
            opMode.telemetry.addLine("!!! INTAKE JAMMED !!!");
        }
    }

    /** Read the sensors without running anything else (e.g. to show the preload during init). */
    public void refreshSensors() {
        readSensors();
    }

    // == Requests =============================================================

    /** Start collecting (stays on while held; stops and rejects by itself when full). */
    public void startIntake() {
        intakeHeld = true;
        jammed = false;
        if (mode != Mode.SHOOTING && mode != Mode.EJECTING && !isFull()) {
            unjamTries = 0;
            setMode(Mode.INTAKING);
        }
    }

    public void stopIntake() {
        intakeHeld = false;
        if (mode == Mode.INTAKING || mode == Mode.UNJAMMING) {
            setMode(Mode.IDLE);
        }
    }

    /** Shoot every ball we hold. */
    public void shootAll() {
        startShooting(false);
    }

    /** Shoot one ball. */
    public void shootOne() {
        startShooting(true);
    }

    public void cancelShoot() {
        if (mode == Mode.SHOOTING) {
            setMode(Mode.IDLE);
        }
    }

    /** Purge every ball out of the intake (overrides everything else). */
    public void startEject() {
        jammed = false;
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
    public int      shotsFired()  { return shotsFired; }
    public boolean  isJammed()    { return jammed; }
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
        String newPattern = sb.toString();
        if (!newPattern.equals(pattern)) {
            patternTimer.reset();
        }
        pattern = newPattern;
        slots = raw;

        // Debounced count.
        int count = rawCount();
        if (count != pendingCount) {
            pendingCount = count;
            countTimer.reset();
        } else if (countTimer.milliseconds() > COUNT_DEBOUNCE_MS) {
            ballCount = count;
        }

        // A ball leaving slot4 while we're pushing through the open gate is a shot.
        boolean slot4 = slots[slots.length - 1];
        if (lastSlot4 && !slot4 && isGateOpen() && feederPower > 0) {
            shotsFired++;
            shotThisFeed = true;
            lastShotTimer.reset();
        }
        lastSlot4 = slot4;
    }

    private int rawCount() {
        int count = 0;
        for (boolean ball : slots) {
            if (ball) count++;
        }
        return count;
    }

    /** True when the balls sit together against the gate (no gaps). */
    private boolean isPacked() {
        boolean seenBall = false;
        for (boolean ball : slots) {
            if (ball) {
                seenBall = true;
            } else if (seenBall) {
                return false;       // a gap after a ball
            }
        }
        return true;
    }

    private boolean isGateOpen() {
        return Math.abs(gatePosition - GATE_OPEN) < Math.abs(gatePosition - GATE_CLOSED);
    }

    // == State machine =========================================================

    private void startShooting(boolean oneOnly) {
        jammed = false;
        if (isEmpty() && rawCount() == 0) {
            return;
        }
        shootOneOnly = oneOnly;
        setMode(Mode.SHOOTING);
        setShotStep(ShotStep.WAIT_READY);
    }

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
                } else if (isPacked()) {
                    unjamTries = 0;
                } else if (patternTimer.milliseconds() > JAM_MS) {
                    if (unjamTries < MAX_UNJAM_TRIES) {
                        unjamTries++;
                        setMode(Mode.UNJAMMING);
                    } else {
                        jammed = true;
                        intakeHeld = false;
                        setMode(Mode.IDLE);
                    }
                }
                break;

            case REJECTING:
                if (modeTimer.milliseconds() > REJECT_MS) {
                    setMode(Mode.IDLE);
                }
                break;

            case UNJAMMING:
                if (modeTimer.milliseconds() > UNJAM_MS) {
                    patternTimer.reset();
                    setMode(intakeHeld ? Mode.INTAKING : Mode.IDLE);
                }
                break;

            case SHOOTING:
                updateShooting();
                break;

            default:
                break;
        }
    }

    private void updateShooting() {
        boolean empty = isEmpty() && rawCount() == 0;

        switch (shotStep) {
            case WAIT_READY:
                if (empty) {
                    setShotStep(ShotStep.CLEARING);
                } else if (shooterReady.getAsBoolean()
                        && lastShotTimer.milliseconds() > MIN_SHOT_GAP_MS) {
                    setShotStep(ShotStep.FEED);
                }
                break;

            case FEED:
                if (shotThisFeed) {
                    shotThisFeed = false;
                    if (shootOneOnly || empty) {
                        setShotStep(ShotStep.CLEARING);
                    } else if (!RAPID_FIRE) {
                        setShotStep(ShotStep.WAIT_READY);
                    } else {
                        modeTimer.reset();          // rapid fire: keep feeding, time the next ball
                    }
                } else if (empty) {
                    setShotStep(ShotStep.CLEARING);
                } else if (modeTimer.milliseconds() > FEED_BALL_TIMEOUT_MS) {
                    jammed = true;
                    setMode(Mode.IDLE);
                }
                break;

            case CLEARING:
                if (lastShotTimer.milliseconds() > SHOT_CLEAR_MS) {
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
        shotThisFeed = false;
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
                if (shotStep == ShotStep.FEED) {
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
            case UNJAMMING:
                set(UNJAM_POWER, UNJAM_POWER, GATE_CLOSED);
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
        if (intake != null) intake.setPower(intakePower);
        if (feeder != null) feeder.setPower(feederPower);
        if (gate != null)   gate.setPosition(gatePosition);
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

    /** Shoot every ball; finishes when the last one has left.  Cancels if interrupted. */
    public Command shootAllCommand() {
        return new FunctionalCommand(
                this::shootAll,
                () -> {},
                interrupted -> { if (interrupted) cancelShoot(); },
                () -> mode != Mode.SHOOTING,
                this);
    }

    /** Shoot one ball; finishes when it has left.  Cancels if interrupted. */
    public Command shootOneCommand() {
        return new FunctionalCommand(
                this::shootOne,
                () -> {},
                interrupted -> { if (interrupted) cancelShoot(); },
                () -> mode != Mode.SHOOTING,
                this);
    }

    /** Purge everything out of the intake while scheduled. */
    public Command ejectCommand() {
        return new StartEndCommand(this::startEject, this::stopEject, this);
    }
}
