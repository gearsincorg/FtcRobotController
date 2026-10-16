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

/**
 * Intake, gate and feeder: collects balls and hands them to the shooter.
 *
 *   - intake motor pulls balls in
 *   - gate servo keeps them out of the shooter until we feed
 *   - feeder motor pushes them into the shooter
 *   - four digital sensors, one per ball slot (slot1 nearest the intake, slot4 nearest the gate)
 *
 * State machine (run in periodic):
 *   EMPTY    -- intake()                      --> INTAKING
 *   INTAKING -- all 4 slots full (debounced)  --> FULL
 *   INTAKING -- stopIntake()                  --> FULL if all 4 slots full, else EMPTY
 *   any      -- feed() (with balls)           --> FEEDING
 *   FEEDING  -- all slots empty (debounced)   --> EMPTY
 *
 * FEEDING keeps the gate closed until the shooter is ready, then opens it and runs the feeder
 * and intake until the slots are empty (or FEED_TIMEOUT_MS passes).
 */
public class IntakeSubsystem extends SubsystemBase {

    // == Tuning ==============================================================
    private static final double INTAKE_POWER      = 1.0;
    private static final double EJECT_POWER       = -1.0;
    private static final double FEED_POWER        = 1.0;

    // TODO tune: gate servo positions.
    private static final double GATE_CLOSED       = 0.0;
    private static final double GATE_OPEN         = 0.5;

    // Sensor reading when a ball is in the slot (most digital sensors are active-low).
    private static final boolean BALL_SEEN_STATE  = false;

    private static final double FULL_DEBOUNCE_MS  = 150;
    private static final double EMPTY_DEBOUNCE_MS = 300;
    private static final double FEED_TIMEOUT_MS   = 2000;   // in case a sensor sticks

    private static final String[] SLOT_NAMES = { "slot1", "slot2", "slot3", "slot4" };

    public enum State { EMPTY, INTAKING, FULL, FEEDING }

    private final OpMode           opMode;
    private final ShooterSubsystem shooter;
    private DcMotorEx        intake;
    private DcMotorEx        feeder;
    private Servo            gate;
    private DigitalChannel[] slotSensors = new DigitalChannel[SLOT_NAMES.length];

    private State   state     = State.EMPTY;
    private boolean[] slots   = new boolean[SLOT_NAMES.length];
    private boolean gateOpen  = false;      // latched once the shooter is ready while FEEDING
    private boolean ejecting  = false;

    private final ElapsedTime fullTimer  = new ElapsedTime();   // time all slots have been full
    private final ElapsedTime emptyTimer = new ElapsedTime();   // time all slots have been empty
    private final ElapsedTime feedTimer  = new ElapsedTime();   // time since the gate opened

    // == standard Methods ===================================
    public IntakeSubsystem(OpMode myOpMode, ShooterSubsystem myShooter) {
        opMode  = myOpMode;
        shooter = myShooter;
    }

    public void init() {
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        feeder = opMode.hardwareMap.get(DcMotorEx.class, "feeder");
        gate   = opMode.hardwareMap.get(Servo.class, "gate");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (int i = 0; i < SLOT_NAMES.length; i++) {
            slotSensors[i] = opMode.hardwareMap.get(DigitalChannel.class, SLOT_NAMES[i]);
            slotSensors[i].setMode(DigitalChannel.Mode.INPUT);
        }

        readSensors();
        setState(isFull() ? State.FULL : State.EMPTY);
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
        StringBuilder pattern = new StringBuilder();
        for (boolean ball : slots) {
            pattern.append(ball ? "X " : "- ");
        }
        opMode.telemetry.addData("INTAKE", "%s  balls = %d  [ %s]", state, ballCount(), pattern);
        opMode.telemetry.addData("  Gate : waiting for shooter", "%s : %b",
                gateOpen ? "OPEN" : "CLOSED", isWaitingForShooter());
    }

    // == new elements =========================================

    /** Start pulling balls in.  Ignored while feeding. */
    public void intake() {
        if (state != State.FEEDING) {
            setState(State.INTAKING);
        }
    }

    /** Stop pulling balls in. */
    public void stopIntake() {
        if (state == State.INTAKING) {
            setState(isFull() ? State.FULL : State.EMPTY);
        }
    }

    /** Feed whatever balls we have into the shooter once it is ready. */
    public void feed() {
        if (state != State.FEEDING && !isEmpty()) {
            setState(State.FEEDING);
        }
    }

    /** Abort feeding and close the gate. */
    public void cancelFeed() {
        if (state == State.FEEDING) {
            setState(isFull() ? State.FULL : State.EMPTY);
        }
    }

    /** Run the intake backwards (clear a jam).  Doesn't change the state. */
    public void eject() {
        ejecting = true;
    }

    public void stopEject() {
        ejecting = false;
    }

    public State getState() {
        return state;
    }

    public int ballCount() {
        int count = 0;
        for (boolean ball : slots) {
            if (ball) count++;
        }
        return count;
    }

    public boolean isFull()  { return ballCount() == slots.length; }
    public boolean isEmpty() { return ballCount() == 0; }

    /** True while FEEDING but the gate is still closed waiting for the shooter. */
    public boolean isWaitingForShooter() {
        return state == State.FEEDING && !gateOpen;
    }

    // == State machine =========================================

    private void readSensors() {
        for (int i = 0; i < slots.length; i++) {
            slots[i] = slotSensors[i].getState() == BALL_SEEN_STATE;
        }
        if (!isFull())  fullTimer.reset();
        if (!isEmpty()) emptyTimer.reset();
    }

    private void updateState() {
        switch (state) {
            case INTAKING:
                if (fullTimer.milliseconds() > FULL_DEBOUNCE_MS) {
                    setState(State.FULL);
                }
                break;

            case FEEDING:
                if (!gateOpen) {
                    if (shooter.isReady()) {
                        gateOpen = true;
                        feedTimer.reset();
                    }
                } else if (emptyTimer.milliseconds() > EMPTY_DEBOUNCE_MS
                        || feedTimer.milliseconds() > FEED_TIMEOUT_MS) {
                    setState(State.EMPTY);
                }
                break;

            default:
                break;
        }
    }

    private void setState(State newState) {
        state    = newState;
        gateOpen = false;
    }

    private void applyOutputs() {
        boolean feeding = state == State.FEEDING && gateOpen;

        double intakePower = (state == State.INTAKING || feeding) ? INTAKE_POWER : 0;
        if (ejecting) {
            intakePower = EJECT_POWER;
        }

        intake.setPower(intakePower);
        feeder.setPower(feeding ? FEED_POWER : 0);
        gate.setPosition(feeding ? GATE_OPEN : GATE_CLOSED);
    }

    // == Command methods  ==========================================

    /** Intake while scheduled (e.g. while a button is held). */
    public Command intakeCommand() {
        return new StartEndCommand(this::intake, this::stopIntake, this);
    }

    /** Intake until all four slots are full (for auto; add a timeout). */
    public Command intakeUntilFullCommand() {
        return new FunctionalCommand(
                this::intake,
                () -> {},
                interrupted -> stopIntake(),
                () -> state == State.FULL,
                this);
    }

    /** Feed the shooter once it is ready; finishes when empty.  Cancels the feed if interrupted. */
    public Command feedCommand() {
        return new FunctionalCommand(
                this::feed,
                () -> {},
                interrupted -> { if (interrupted) cancelFeed(); },
                () -> state != State.FEEDING,
                this);
    }

    /** Run the intake backwards while scheduled. */
    public Command ejectCommand() {
        return new StartEndCommand(this::eject, this::stopEject);
    }
}
