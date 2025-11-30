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
    private boolean newBall = false;

    // Subsystem Constants
    private final double INTAKE_POWER = 1.0;

    private final double PULSE_SCALE_FACTOR = 3.125e-3;  // CONVERTS 320 DEG TO 1.0 range ??
    private final double MIN_RANGE =  50;
    private final double MAX_RANGE = 120;

    // Flipper Servo positions and times for shooting
    private final double FIRE_SHOOT     = 0.65;
    private final double FIRE_RETRACT   = 0.12;

    private final double FIRE_HOLD_TIME         = 0.25;
    private final double ADVANCE_DELAY_TIME     = 0.20;  // was 0.15
    private final double NEW_ARTIFACT_HOLD_TIME = 0.10;  // was 0.02

    // Spindexer Servo Positions (in degrees)
    private final double   CENTER_OFFSET = 5.0;
    private final double[] SHOOT        = {-120,   0,  120};  // adjust for offcenter allignment
    private final double[] INTAKE_FRONT = { -30,  90, -150};
    private final double[] INTAKE_BACK  = { 150, -90,   30};
    private final double[] HOME_ANGLES  = { 120,   0, -120};
    private final int[][]  AUTO_SLOTS   = {{2, 1, 0}, {0, 2, 1}, {0, 1, 2}};

    // General Subsystem Members
    private double intakePower          =  0;
    private double targetAngle          = -1;
    private double currentAngle         =  0;
    private double estimatedTransitTime =  0;
    private double lastSpindexerServoValue = 0;
    private ElapsedTime spinServoTimer  =  new ElapsedTime();
    private double sensorRange          =  0;

    private boolean shootingPreloads = true;
    private int patternID           = 2;
    private int currentSlot         = 0;
    private int allArtifactsHeld    = 0;
    private int greenArtifactsHeld  = 0;
    private int purpleArtifactsHeld = 0;
    private int currentAutoSlot     = 0;

    private boolean startAutoShoot  = false;

    private ArtifactColor currentColor   = ArtifactColor.UNKNOWN;
    private ArtifactColor queuedColor  = ArtifactColor.ANY;
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
        // Read the spindexer position and determine which segment and slot we are in.
        // SharedOQ.update();
        if ((currentState == INTAKING) && inPosition() && (slotColors[currentSlot] == ArtifactColor.UNKNOWN)) {
            if (Globals.FORWARD_MOTION) {
                sensorRange = distanceFront.getDistance(DistanceUnit.MM);
                myOpMode.telemetry.addData("distance front = %.0f", sensorRange);
                if ((sensorRange > MIN_RANGE) && (sensorRange < MAX_RANGE)) {
                    slotColors[currentSlot] = ArtifactColor.PURPLE;
                    newBall = true;
                }
            } else {
                sensorRange = distanceBack.getDistance(DistanceUnit.MM);
                myOpMode.telemetry.addData("distance back = %.0f", sensorRange);
                if (distanceBack.getDistance(DistanceUnit.MM) > MIN_RANGE && distanceBack.getDistance(DistanceUnit.MM) < MAX_RANGE) {
                    slotColors[currentSlot] = ArtifactColor.PURPLE;
                    newBall = true;
                }
            }
        }

        // count number of slots with balls.
        int purpleCount = 0;
        int greenCount = 0;
        for (int b = 0; b < 3; b++) {
            if (slotColors[b] == ArtifactColor.GREEN) {
                greenCount++;
            } else if (slotColors[b] == ArtifactColor.PURPLE) {
                purpleCount++;
            }
        }

        allArtifactsHeld    = greenCount + purpleCount;
        greenArtifactsHeld  = greenCount;
        purpleArtifactsHeld = purpleCount;
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
                //sendToShooter(1); //might be used
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
                newBall = false;  // reset this for next intake
                if (inPosition())   {
                    runIntake();
                    setState(INTAKING);
                }
                break;
            }

            case INTAKING: {
                if (newBall) {
                    setState(INTAKE_HOLD);
                } if (Globals.ROBOT_STATE == RobotStates.SHOOTING) {
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
                        myOpMode.gamepad1.leftBumperWasPressed();  // forget any past button presses
                        sendToShooter(0);                      // queue up first shot.
                        setState(SHOT_QUEUEING);
                    } else {
                        stopIntake();
                        sendClostestEmptyToIntake();
                        setState(INTAKE_QUEUEING);
                    }
                }
                break;
            }

            case SHOT_QUEUEING: {
                newBall = false;  // reset this for next intake
                if (allArtifactsHeld == 0 ) {
                    setState(INTAKING);
                } else if (inPosition())   {
                    setState(READY_TO_SHOOT);
                }
                break;
            }

            case READY_TO_SHOOT: {
                if ((myOpMode.gamepad1.right_bumper || myOpMode.gamepad1.leftBumperWasPressed() || startAutoShoot) &&
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
        }

        // Save current state in Globals for other subsystems
        Globals.SPINDEXER_STATE = (SpindexerStates) currentState;
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("INTAKE", "Pwr %.1f", intakePower);
        myOpMode.telemetry.addData("SPINDEX", "%s (s%d) -> %.1f %s (%.2f)", currentState, currentSlot, targetAngle, inPosition(), lastSpindexerServoValue);
        myOpMode.telemetry.addData("SLOTS", "%s %s %s", slotColors[0], slotColors[1], slotColors[2]);
        if (Globals.IS_AUTO) {
            myOpMode.telemetry.addData("AUTO", "%s", startAutoShoot ? "Auto Shoot Active" : "idle");
        }

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
        } else {
            destination = -90;
        }

        // calculate how far the spindexer needs to turn for each empty slot, and use the smallest angle.
        for(int s = 0; s < 3; s++){
            if (slotColors[s] == ArtifactColor.UNKNOWN) {
                double angle = Math.abs(destination - currentAngle - HOME_ANGLES[s]);
                if (angle < closestAngle) {
                    closestAngle = angle;
                    closestSlot = s;
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
        double destination  =   0;

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
    }

    public void sendToIntake(int slot){
        if (Globals.FORWARD_MOTION){
            sendToAngle(INTAKE_FRONT[slot]);
        } else {
            sendToAngle(INTAKE_BACK[slot]);
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

    private void sendToAngle(double newTargetAngle){
        // only process new targets
        if (newTargetAngle != targetAngle ) {
            lastSpindexerServoValue = MathUtils.clamp(0.5 + ((newTargetAngle + CENTER_OFFSET) * PULSE_SCALE_FACTOR), 0, 1.0);
            spindexer.setPosition(lastSpindexerServoValue);
            estimatedTransitTime = Math.abs((newTargetAngle - targetAngle)) / 450; // SWYFT torque servo .. 60 deg in .115 sec = 514 deg/s
            spinServoTimer.reset();
            targetAngle = newTargetAngle ;
        }
    }

    /**
     * Convert any angle to a +/- 180  degree value.
     * @param angle
     * @return
     */
    double normalizeAngle(double angle){
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }

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
}
