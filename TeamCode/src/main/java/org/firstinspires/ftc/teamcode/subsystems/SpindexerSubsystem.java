package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.*;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {

    public SpindexerSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private DcMotor intake;
    private Servo fire;
    private Servo spindexer;
    private Rev2mDistanceSensor distanceFront;
    private Rev2mDistanceSensor distanceBack;
    private boolean newArtifact = false;

    // Subsystem Constants
    private final double INTAKE_POWER =  0.9;  // a bit slower TEST
    private final double HOLD_POWER   =  0.4;  // a bit slower TEST
    private final double EJECT_POWER  = -0.7;  // a bit faster TEST

    // private final double PULSE_SCALE_FACTOR = 1.0 / 1620.0;  // CONVERTS 1620 DEG TO 1.0 range ??
    private final double PULSE_SCALE_FACTOR = 0.299 / 480.0;  // CONVERTS 960 DEG TO 0.595range ??
    private final double CENTER_OFFSET = 0; // -3.24;  // used to adjust the spindexer so 0 deg is B centered
    private final double MIN_RANGE =  50; // was 50
    private final double MAX_RANGE = 120; // was 120

    // Flipper Servo positions and times for shooting
    private final double FIRE_SHOOT     = 0.65;
    private final double FIRE_RETRACT   = 0.12;

    private final double FIRE_HOLD_TIME         = 0.25;
    private final double ADVANCE_DELAY_TIME     = 0.25;  // was 0.15
    private final double NEW_ARTIFACT_HOLD_TIME = 0.30;  // was 0.02

    // Spindexer Servo Positions (in degrees)
    private final double[] SHOOT        = {-120,   0,  120};
    private final double[] INTAKE_FRONT = { -30,  90,  210};
    private final double[] INTAKE_BACK  = {-210, -90,   30};
    private final double[] HOME_ANGLES  = { 120,   0, -120};
    private final int[][]  AUTO_SLOTS   = {{2, 1, 0}, {0, 2, 1}, {0, 1, 2}};

    private final double[] REFINED_SHOOT = {0.577, 0.502, 0.421};
    private final double[] REFINED_FRONT = {0.520, 0.442, 0.366};
    private final double[] REFINED_BACK  = {0.630, 0.557, 0.480};

    // General Subsystem Members
    private double intakePower          =  0;
    private double targetAngle          = -1;
    private double currentAngle         =  0;
    private double estimatedTransitTime =  0;
    private double spindexerServoValue = 0;
    private boolean lastDirectionForward = true;
    private ElapsedTime spinServoTimer  =  new ElapsedTime();
    private double sensorRange          =  0;

    private boolean startAutoShoot      = false;
    private boolean shootingPreloads    = true;
    private int     patternID           = 2;
    private int     currentSlot         = 0;
    private int     allArtifactsHeld    = 0;
    private int     currentAutoSlot     = 0;
    private double  lastSlotAngleFilled = 0;  // Used during unjamming
    private int     lastSlotFilled      = -1;
    private boolean unjamForward        = false;

    private ArtifactColor[] slotColors = {ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN};

    @Override
    public void init (boolean showTelemetry) {

        super.init(showTelemetry);  // do not remove
        setState(INIT);

        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Attach to physical devices and configure them
        fire = myOpMode.hardwareMap.get(Servo.class, "fire");
        fire.setPosition(FIRE_RETRACT);

        spindexer = myOpMode.hardwareMap.get(Servo.class, "spindexer");
        distanceFront = myOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "distanceFront");
        distanceBack = myOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "distanceBack");

        spinServoTimer.reset();
    }

    @Override
    /**
     * Read any sensor for this subsystem and calculate any derived values
     * Called every Update() cycle;
     */
    public void readSensors() {
        // process the artifact range sensors if we are INTAKING
        if (currentState == INTAKING) {

            // Watch for a direction change...  set new position if needed
            if (lastDirectionForward != Globals.FORWARD_MOTION ) {
                sendClostestEmptyToIntake();
                lastDirectionForward =  Globals.FORWARD_MOTION;
            }

            // check contents if we are presenting an empty slot
            if (inPosition() && (slotColors[currentSlot] == ArtifactColor.UNKNOWN)) {
                // check the appropriate sensor based on travel direction
                if (Globals.FORWARD_MOTION) {
                    sensorRange = distanceFront.getDistance(DistanceUnit.MM);
                    myOpMode.telemetry.addData("distance front = %.0f", sensorRange);
                } else {
                    sensorRange = distanceBack.getDistance(DistanceUnit.MM);
                    myOpMode.telemetry.addData("distance back = %.0f", sensorRange);
                }

                // see if we have an artifact
                if ((sensorRange > MIN_RANGE) && (sensorRange < MAX_RANGE)) {
                    slotColors[currentSlot] = ArtifactColor.PURPLE;

                    lastSlotFilled = currentSlot;
                    lastSlotAngleFilled = targetAngle; // Save current location

                    newArtifact = true;
                }
            }
        }

        // count number of slots with balls.
        allArtifactsHeld = 0;
        for (int b = 0; b < 3; b++) {
            if (slotColors[b] != ArtifactColor.UNKNOWN) {
                allArtifactsHeld++;
            }
        }
    }

    @Override
    /**
     *  Run any non-state machine pre-processing
     *  Called every update() Cycle
     */
    public void runProcessing() {
    }

    @Override
    /**
     *  Run subsystem state machine
     *  Called every update() Cycle
     */
    public void runStateMachine() {
        switch ((SpindexerStates)currentState) {
            case INIT: {
                sendToIntake(0); //might be used
                setState(HOME);
                break;
            }

            case HOME: {
                if (myOpMode.opModeIsActive()) {
                    if (allArtifactsHeld == 3) {
                        // sending the spindexer to the position needed fpr the first color of the obolisk pattern
                        sendToShooter(AUTO_SLOTS[patternID][currentAutoSlot++]);
                        setState(SHOT_QUEUEING);
                    } else {
                        sendClostestEmptyToIntake();
                        setState(INTAKE_QUEUEING);
                    }
                }
                break;
            }

            case INTAKE_QUEUEING: {
                newArtifact = false;  // reset this for next intake
                if (inPosition())   {
                    runIntake();
                    setState(INTAKING);
                }
                break;
            }

            case INTAKING: {
                if (myOpMode.gamepad1.right_trigger > 0.25) {
                    ejectIntake(); // Start UNJAM process
                    sendToLastAngle(lastSlotAngleFilled, lastSlotFilled);
                    setState(BASIC_UNJAM);
                } else if (myOpMode.gamepad1.left_trigger > 0.25) {
                    ejectIntake(); // Start LUDICROUS UNJAM process
                    setState(LUDICROUS_UNJAM);
                } else if (myOpMode.gamepad1.left_bumper) {
                    stopIntake();  // Force shooting even without 3 artifacts
                    Globals.ROBOT_STATE = RobotStates.SHOOTING;
                    myOpMode.gamepad1.leftBumperWasPressed();  // forget any past button presses
                    sendClostestColorToShooter(ArtifactColor.ANY);                    // queue up first shot.
                    setState(SHOT_QUEUEING);
                } else if (newArtifact) {
                    setState(INTAKE_HOLD);
                } else if (Globals.ROBOT_STATE == RobotStates.SHOOTING) {
                    stopIntake();
                    sendClostestColorToShooter(ArtifactColor.ANY);
                    setState(SHOT_QUEUEING);
                } else {
                    sendClostestEmptyToIntake();
                }
                break;
            }

            case INTAKE_HOLD: {
                if (timeInState(NEW_ARTIFACT_HOLD_TIME)) {
                    if (allArtifactsHeld == 3) {
                        stopIntake();
                        Globals.ROBOT_STATE = RobotStates.SHOOTING;
                        sendClostestColorToShooter(ArtifactColor.ANY);
                        // sendToShooter(0);                      // queue up first shot.
                        setState(SHOT_QUEUEING);
                    } else {
                        holdInIntake();
                        sendClostestEmptyToIntake();
                        setState(INTAKE_QUEUEING);
                    }
                }
                break;
            }

            case SHOT_QUEUEING: {
                newArtifact = false;  // reset this for next intake
                if (allArtifactsHeld == 0 ) {
                    Globals.ROBOT_STATE = RobotStates.INTAKING;
                    setState(INTAKE_QUEUEING);
                } else if (inPosition())   {
                    setState(READY_TO_SHOOT);
                }
                break;
            }

            case READY_TO_SHOOT: {
                if (myOpMode.gamepad1.right_trigger > 0.25) {
                    ejectIntake();
                    sendToLastAngle(lastSlotAngleFilled, lastSlotFilled);
                    Globals.ROBOT_STATE = RobotStates.INTAKING;
                    setState(BASIC_UNJAM);
                } else if (myOpMode.gamepad1.left_trigger > 0.25) {
                    ejectIntake(); // Start LUDICROUS UNJAM process
                    setState(LUDICROUS_UNJAM);
                } else if ((myOpMode.gamepad1.right_bumper || startAutoShoot) &&
                    Globals.SHOOTER_AT_SPEED && Globals.TURRET_ON_TARGET) {
                    fire.setPosition(FIRE_SHOOT);
                    slotColors[currentSlot] = ArtifactColor.UNKNOWN;
                    setState(SHOOTING);
                }
                break;
            }

            case SHOOTING: {
                if (timeInState(FIRE_HOLD_TIME)) {
                    fire.setPosition(FIRE_RETRACT);
                    setState(COCKING_SHOT);
                }
                break;
            }

            case COCKING_SHOT: {
                if (timeInState(ADVANCE_DELAY_TIME)) {
                    if (allArtifactsHeld > 0) {
                        // only run the preload sequence once.  Then just get the next available artifact
                        if (Globals.IS_AUTO && shootingPreloads) {
                            sendToShooter(AUTO_SLOTS[patternID][currentAutoSlot++]);
                        } else {
                            sendClostestColorToShooter(ArtifactColor.ANY);
                        }
                        setState(SHOT_QUEUEING);
                    } else {
                        currentAutoSlot = 0; //after shooting if we intake three more it needs to reset
                        shootingPreloads = false;  // we are done with preloads.
                        startAutoShoot = false;
                        runIntake();
                        Globals.ROBOT_STATE = RobotStates.INTAKING;
                        setState(INTAKING);
                    }
                }
                break;
            }

            case BASIC_UNJAM: {
                if (myOpMode.gamepad1.right_trigger < 0.25){
                    stopIntake();
                    if (allArtifactsHeld == 3) {
                        sendClostestColorToShooter(ArtifactColor.ANY);
                        Globals.ROBOT_STATE = RobotStates.SHOOTING;
                        setState(SHOT_QUEUEING);
                    } else {
                        setState(INTAKE_QUEUEING);
                    }
                }
                break;
            }

            case LUDICROUS_UNJAM: {
                if (myOpMode.gamepad1.left_trigger < 0.25){
                    stopIntake();
                    slotColors[0] = ArtifactColor.UNKNOWN;
                    slotColors[1] = ArtifactColor.UNKNOWN;
                    slotColors[2] = ArtifactColor.UNKNOWN;
                    sendToIntake(0);
                    Globals.ROBOT_STATE = RobotStates.INTAKING;
                    setState(INTAKE_QUEUEING);
                } else {
                    // shake the ball loose
                    if (timeInState(0.2)) {
                        spindexer.setPosition(unjamForward ? 0.6 : 0.4);
                        unjamForward = !unjamForward;
                        setState(LUDICROUS_UNJAM);  // Start the state timer running again.
                    }
                }
                break;
            }
        }

        // Save current state in Globals for other subsystems
        Globals.SPINDEXER_STATE = (SpindexerStates) currentState;
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("INTAKE", "Pwr %.1f", intakePower);
        myOpMode.telemetry.addData("SPINDEX", "%s (s%d)->%.1f %s (%.3f)", currentState, currentSlot, targetAngle, inPosition(), spindexerServoValue);
        myOpMode.telemetry.addData("SLOTS", "%s %s %s", slotColors[0], slotColors[1], slotColors[2]);
    }

    public void startIntaking() {
        sendClostestEmptyToIntake();
        Globals.ROBOT_STATE = RobotStates.INTAKING;
        setState(INTAKE_QUEUEING);
    }

    public void startShooting() {
        sendClostestColorToShooter(ArtifactColor.ANY);
        Globals.ROBOT_STATE = RobotStates.SHOOTING;
        setState(SHOT_QUEUEING);
    }

    public void runIntake(){
        intakePower = INTAKE_POWER;
        intake.setPower(intakePower);
    }

    public void holdInIntake(){
        intakePower = HOLD_POWER;
        intake.setPower(intakePower);
    }

    public void ejectIntake(){
        intakePower = EJECT_POWER;
        intake.setPower(intakePower);
    }

    public void stopIntake(){
        intakePower = 0.0;
        intake.setPower(intakePower);
    }


    /**
     * sends the best empty slot to the intake by deciding on the smallest distance between the three.
     */
    private void sendClostestEmptyToIntake(){
        double closestAngle = 360;
        int    closestSlot  =   -1;
        double destination;

        if (Globals.FORWARD_MOTION){
            destination = 90;
            for(int s = 0; s < 3; s++){
                if (slotColors[s] == ArtifactColor.UNKNOWN) {
                    double angle = Math.abs(destination - currentAngle - HOME_ANGLES[s]);
                    if (angle < closestAngle) {
                        closestAngle = angle;
                        closestSlot = s;
                    }
                }
            }
        } else {
            destination = -90;
            for(int s = 2; s >= 0; s--){
                if (slotColors[s] == ArtifactColor.UNKNOWN) {
                    double angle = Math.abs(destination - currentAngle - HOME_ANGLES[s]);
                    if (angle < closestAngle) {
                        closestAngle = angle;
                        closestSlot = s;
                    }
                }
            }
        }

        if (closestSlot >= 0){
            sendToIntake(closestSlot);
        }
    }

    /**
     * sends the best full slot to the intake by deciding on the smallest distance between the three.
     */
    private void sendClostestColorToShooter(ArtifactColor color){
        double closestAngle = 360;
        int    closestSlot  =  -1;
        double destination  =   0;  // can be simplified

        // calculate how far the spindexer needs to turn for each full slot, and use the smallest angle.
        for (int s = 0; s < 3; s++) {
            // do a color match or a match all
            if ((slotColors[s] == color) || ((color == ArtifactColor.ANY) && (slotColors[s] != ArtifactColor.UNKNOWN))) {
                double angle = Math.abs(destination - currentAngle - HOME_ANGLES[s]);
                if (angle < closestAngle) {
                    closestAngle = angle;
                    closestSlot = s;
                }
            }
        }

        if (closestSlot >= 0){
            sendToShooter(closestSlot);
        }
    }

    public void sendToShooter(int slot) {
        sendToAngle(SHOOT[slot]);
        currentSlot = slot;

        // refine the servo position.
        spindexerServoValue = REFINED_SHOOT[slot];
        spindexer.setPosition(spindexerServoValue);
    }


    public void sendToIntake(int slot){
        if (Globals.FORWARD_MOTION){
            sendToAngle(INTAKE_FRONT[slot]);
            // refine the servo position.
            spindexerServoValue = REFINED_FRONT[slot];
            spindexer.setPosition(spindexerServoValue);
        } else {
            sendToAngle(INTAKE_BACK[slot]);
            // refine the servo position.
            spindexerServoValue = REFINED_BACK[slot];
            spindexer.setPosition(spindexerServoValue);
        }
        currentSlot = slot;
    }

    public boolean inPosition() {
        if (spinServoTimer.time() > estimatedTransitTime) {
          currentAngle = targetAngle;
          return true;
        }
        else {
          return false;
        }
    }

    private void sendToLastAngle(double lastAngle, int lastSlot) {
        currentSlot = MathUtils.clamp(lastSlot, 0, 2);
        sendToAngle(lastAngle);
    }

    private void sendToAngle(double newTargetAngle){
        // only process new targets
        if (newTargetAngle != targetAngle ) {
            spindexerServoValue = MathUtils.clamp(0.502 - ((newTargetAngle + CENTER_OFFSET) * PULSE_SCALE_FACTOR), 0, 1.0);
            spindexer.setPosition(spindexerServoValue);
            estimatedTransitTime = Math.abs((newTargetAngle - targetAngle)) / 270; //
            spinServoTimer.reset();
            targetAngle = newTargetAngle ;
        }
    }

    /**
     * Convert any angle to a +/- 180  degree value.
     * @param angle
     * @return
     */
    private double normalizeAngle(double angle){
        while (angle > 180) { angle -= 360; }
        while (angle < -180) { angle += 360; }
        return angle;
    }

    public void preloadSequence(){
        slotColors[0] = ArtifactColor.PURPLE;
        slotColors[1] = ArtifactColor.PURPLE;
        slotColors[2] = ArtifactColor.GREEN;
    }

    public void setPatternID (int id){
        patternID = id;
    }

    // =============  Action methods  ========================

    public Action actionWaitForState(SpindexerStates state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                return currentState != state;
            }
        };
    }

    public Action actionStartAutoShooting(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                startAutoShoot = true;
                return false;
            }
        };
    }

    public Action actionStopIntake(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                stopIntake();
                return false;
            }
        };
    }
}
