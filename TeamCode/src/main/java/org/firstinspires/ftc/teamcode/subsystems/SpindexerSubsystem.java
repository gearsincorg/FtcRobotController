package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.*;

import android.graphics.Color;

import androidx.core.math.MathUtils;

import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {

    public SpindexerSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private IntakeSubsystem  intake  = new IntakeSubsystem(myOpMode);

    private Servo fire;
    private Servo spindexer;
    private NormalizedColorSensor frontColorSensor;
    private NormalizedColorSensor backColorSensor;
    private boolean newBall = false;

    // Subsystem Constants
    private final double PULSE_SCALE_FACTOR = 1.8e-3;  // CONVERTS 150 DEG TO 0.28 ??

    // Color match constants
    private final double MIN_SATURATION = 0.1;
    private final double MAX_SATURATION = 0.9;
    private final float  COLOR_GAIN     = 3.0f;
    private final double GREEN_MIN      = 122.0;
    private final double GREEN_MAX      = 165.0;
    private final double PURPLE_MIN     = 220.0;
    private final double PURPLE_MAX     = 300.0;

    // Flipper Servo positions and times for shooting
    private final double FIRE_SHOOT     = 0.65;
    private final double FIRE_RETRACT   = 0.085;

    private final double FIRE_HOLD_TIME = 0.2;
    private final double ADVANCE_DELAY_TIME = 0.05;
    private final double NEW_ARTIFACT_HOLD_TIME = 0.1;

    // Spindexer Servo Positions (in degrees)
    private final double[] SHOOT        = {-120,   0,  120};
    private final double[] INTAKE_FRONT = { -30,  90, -150};
    private final double[] INTAKE_BACK  = { 150, -90,   30};
    private final double[] HOME_ANGLES  = { 120,   0, -120};

    // General Subsystem Members
    private double targetAngle      = 0;
    private double currentAngle     = 0;
    private double estimatedTransitTime = 0;
    private double lastSpindexerServoValue = 0;
    private ElapsedTime spinServoTimer = new ElapsedTime();

    private int     currentSlot     = 0;
    private float[] hsvValues = new float[3];
    private int allArtifactsHeld = 0;
    private int greenArtifactsHeld = 0;
    private int purpleArtifactsHeld = 0;

    private ArtifactColor currentColor   = ArtifactColor.UNKNOWN;
    private ArtifactColor queuedColor  = ArtifactColor.ANY;
    private ArtifactColor[] slotColors = {ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN};

    @Override
    public void init (boolean showTelemetry) {

        super.init(showTelemetry);  // do not remove
        setState(INIT);

        // Attach to physical devices and configure them
        fire = myOpMode.hardwareMap.get(Servo.class, "fire");
        fire.setPosition(FIRE_RETRACT);

        spindexer = myOpMode.hardwareMap.get(Servo.class, "spindexer");

        frontColorSensor = myOpMode.hardwareMap.get(ColorRangeSensor.class, "colorFront");
        frontColorSensor.setGain(COLOR_GAIN);

        backColorSensor = myOpMode.hardwareMap.get(ColorRangeSensor.class, "colorBack");
        backColorSensor.setGain(COLOR_GAIN);

        spinServoTimer.reset();

        // initialize all the subsystem
        intake.init(true);
    }

    @Override
    public void update() {
        super.update();  // do not remove
        intake.update();
    }

    @Override
    /**
     * Read any sensor for this subsystem and calculate any derived values
     * Called every Update() cycle;
     */
    public void readSensors() {
        // Read the spindexer position and determine which segment and slot we are in.
        // SharedOQ.update();

        NormalizedRGBA colors;
        if ((currentState == INTAKING) && inPosition() && (slotColors[currentSlot] == ArtifactColor.UNKNOWN)){
            if (Globals.FORWARD_MOTION){
                // front intake
                colors = frontColorSensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
                myOpMode.telemetry.addData("Front HSV", "%s %s %s", hsvValues[0], hsvValues[1], hsvValues[2]);
            } else {
                // back intake
                colors = backColorSensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
                myOpMode.telemetry.addData("Back  HSV", "%s %s %s", hsvValues[0], hsvValues[1], hsvValues[2]);
            }

            // checking the hue and saturation of the color sensor.
            // saturation needs to be high enough use the hue value, but not 1.0
            // find which range the hue resides in to decide the color
            double saturation =  hsvValues[1];
            if ((saturation > MIN_SATURATION) && (saturation < MAX_SATURATION)) {
                if ((hsvValues[0] > GREEN_MIN) && (hsvValues[0] < GREEN_MAX)) {
                    currentColor  = ArtifactColor.GREEN;
                    slotColors[currentSlot] = currentColor;
                    newBall = true;
                } else if ((hsvValues[0] > PURPLE_MIN) && (hsvValues[0] < PURPLE_MAX)) {
                    currentColor = ArtifactColor.PURPLE;
                    slotColors[currentSlot] = currentColor;
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
                sendToShooter(0);
                setState(HOME);
                break;
            }

            case HOME: {
                if (myOpMode.opModeIsActive()) {
                    if (allArtifactsHeld == 3) {
                        sendToShooter(0);
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
                    intake.startIntaking();
                    setState(INTAKING);
                }
                break;
            }

            case INTAKING: {
                if (newBall) {
                    setState(INTAKE_HOLD);
                } else {
                    sendClostestEmptyToIntake();
                }

                break;
            }

            case INTAKE_HOLD: {
                if (timeInState(NEW_ARTIFACT_HOLD_TIME)) {
                    if (allArtifactsHeld == 3) {
                        intake.stopIntaking();
                        myOpMode.gamepad1.leftBumperWasPressed();  // forget any past button presses
                        sendToShooter(0);                      // queue up first shot.
                        setState(SHOT_QUEUEING);
                    } else {
                        // intake.stopIntaking();    // Only needed if balls jam during intake spin.
                        sendClostestEmptyToIntake();
                        setState(INTAKE_QUEUEING);
                    }
                }
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
                if(myOpMode.gamepad1.yWasPressed()) {
                    sendClostestColorToShooter(ArtifactColor.PURPLE);
                    setState(SHOT_QUEUEING);
                }  else if(myOpMode.gamepad1.xWasPressed()) {
                    sendClostestColorToShooter(ArtifactColor.GREEN);
                    setState(SHOT_QUEUEING);
                }  else  if ((myOpMode.gamepad1.right_bumper || myOpMode.gamepad1.leftBumperWasPressed()) && Globals.AT_SPEED) {
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
                //if (timeInState(ADVANCE_DELAY_TIME)) {
                if (allArtifactsHeld > 0) {
                    sendClostestColorToShooter(ArtifactColor.ANY);
                    setState(SHOT_QUEUEING);
                } else {
                    setState(INTAKING);
                }
                //}
                break;
            }
        }

        // Save current state in Globals for other subsystems
        Globals.SPINDEXER_STATE = (SpindexerStates) currentState;
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("Spin", "%s (s%d) %.1f -> %.1f %s (%.2f)", currentState, currentSlot, currentAngle,targetAngle, inPosition(), lastSpindexerServoValue);
        myOpMode.telemetry.addData("Slots", "%s %s %s", slotColors[0], slotColors[1], slotColors[2]);
        myOpMode.telemetry.addData("Forward motion", "%s", Globals.FORWARD_MOTION);
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
            lastSpindexerServoValue = MathUtils.clamp(0.5 + (newTargetAngle * PULSE_SCALE_FACTOR), 0.22, 0.78);
            spindexer.setPosition(lastSpindexerServoValue);
            estimatedTransitTime = Math.abs((newTargetAngle - targetAngle)) / 360; // SWYFT torque servo .. 60 deg in .115 sec
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
        slotColors[3] = ArtifactColor.GREEN;
    }
}
