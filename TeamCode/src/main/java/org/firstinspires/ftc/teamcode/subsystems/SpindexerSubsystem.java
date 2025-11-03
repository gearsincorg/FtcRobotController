package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.subsystems.SpindexerStates.*;

import org.firstinspires.ftc.teamcode.auxtools.SharedOQ;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {

    public SpindexerSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private Servo fire;
    private Servo spindexer;
    private NormalizedColorSensor frontColorSensor;
    private NormalizedColorSensor backColorSensor;

    // Subsystem Constants
    private final int    OQ_ENCODER_INDEX = 2;
    private final double ENC_TO_DEGREES   = 360.0 / 8192.0;
    private final double POSITION_TOLLERANCE = 5;
    private final double COLOR_SENSOR_POSITION_TOLLERANCE = 30;
    private final double PULSE_SCALE_FACTOR = 1.8e-3;  // CONVERTS 150 DEG TO 0.28

    // Color match constants
    private final double MIN_SATURATION = 0.1;
    private final float  COLOR_GAIN     = 5;
    private final double GREEN_MIN      = 120.0;
    private final double GREEN_MAX      = 165.0;
    private final double PURPLE_MIN     = 220.0;
    private final double PURPLE_MAX     = 300.0;

    // Flipper Servo positions and times for shooting
    private final double FIRE_RETRACT   = 0.085;
    private final double FIRE_SHOOT     = 0.50;

    private final double ADVANCE_DELAY_TIME = 0.05;
    private final double FIRE_HOLD_TIME = 0.15;

    // Spindexer Servo Positions (in degrees)
    private final double[] SHOOT        = {-120,   0,  120};
    private final double[] INTAKE_FRONT = { -30,  90, -150};
    private final double[] INTAKE_BACK  = { 150, -90,   30};
    private final double[] HOME_ANGLES  = { 120,   0, -120};

    // General Subsystem Members
    private double currentAngle    = 0;
    private double targetAngle     = 0;
    private double lastTargetAngle = 0;
    private boolean inPosition     = false;
    private boolean nearPosition   = false;
    private double lastSpindexerServoValue = 0;

    private int     currentSlot     = 0;
    private float[] hsvValues = new float[3];
    private int allArtifactsHeld = 0;
    private int greenArtifactsHeld = 0;
    private int purpleArtifactsHeld = 0;

    private ArtifactColor currentColor   = ArtifactColor.UNKNOWN;
    private ArtifactColor queuedColor  = ArtifactColor.ANY;
    private ArtifactColor[] slotColors = {ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.UNKNOWN};
    //private ArtifactColor[] slotColors = {ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN};

    @Override
    public void init (boolean showTelemetry) {

        super.init(showTelemetry);  // do not remove
        setState(INIT);

        // Attach to physical devices and configure them
        fire = myOpMode.hardwareMap.get(Servo.class, "fire");
        fire.setPosition(FIRE_RETRACT);

        spindexer = myOpMode.hardwareMap.get(Servo.class, "spindexer");
        sendSpindexerTo(SHOOT[1]);

        //frontColorSensor = myOpMode.hardwareMap.get(ColorRangeSensor.class, "colorFront");
        //frontColorSensor.setGain(COLOR_GAIN);

        //backColorSensor = myOpMode.hardwareMap.get(ColorRangeSensor.class, "colorBack");
        //backColorSensor.setGain(COLOR_GAIN);
    }


    @Override
    public void readSensors() {
        // Read the spindexer position and determine which segment and slot we are in.
        if (SharedOQ.OQencoder.isDataValid()) {
            currentAngle   = (double)(SharedOQ.OQencoder.positions[OQ_ENCODER_INDEX]) * ENC_TO_DEGREES;
        }

        inPosition = Math.abs(targetAngle - currentAngle) <= POSITION_TOLLERANCE;
        nearPosition = Math.abs(targetAngle - currentAngle) <= COLOR_SENSOR_POSITION_TOLLERANCE;
        NormalizedRGBA colors;

        if ((currentState == INTAKING) && nearPosition){
            if (Globals.AXIAL_MOTION >= 0){
                // front intake
                //colors = frontColorSensor.getNormalizedColors();
            } else {
                // back intake
                //colors = backColorSensor.getNormalizedColors();
            }

            //Color.colorToHSV(colors.toColor(), hsvValues);

            //checking the hue and saturation of the color sensor
            //saturation needs to be high enough use the hue value
            //find which range the hue resides in to decide the color
            if (hsvValues[1] > MIN_SATURATION) {
                if ((hsvValues[0] > GREEN_MIN) && (hsvValues[0] < GREEN_MAX)) {
                    currentColor  = ArtifactColor.GREEN;
                    slotColors[currentSlot] = currentColor;
                } else if ((hsvValues[0] > PURPLE_MIN) && (hsvValues[0] < PURPLE_MAX)) {
                    currentColor = ArtifactColor.PURPLE;
                    slotColors[currentSlot] = currentColor;
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
    public void runStateMachine() {
        switch ((SpindexerStates)currentState) {
            case INIT: {
                sendToShooter(1);
                setState(HOMING);
                break;
            }

            case HOMING: {
                if (timeInState(0.25)) {
                    SharedOQ.resetEncoder(OQ_ENCODER_INDEX);
                    setState(HOME);
                }
                break;
            }

            case HOME: {
                if (myOpMode.opModeIsActive()) {
                    setState(INTAKING);
                }
                break;
            }

            case INTAKING: {
                // THIS if JUST FOR TESTING
                if (myOpMode.gamepad1.dpadLeftWasPressed()){
                    slotColors[2] = ArtifactColor.PURPLE;
                }
                // THIS if JUST FOR TESTING

                if (allArtifactsHeld == 3) {
                    sendToShooter(0);
                    setState(QUEUEING);

                }
                sendClostestEmptyToIntake();
                break;
            }

            case QUEUEING: {
                if (allArtifactsHeld == 0 ) {
                    setState(INTAKING);
                } else if (inPosition)   {
                    setState(READY_TO_SHOOT);
                }
                break;
            }

            case READY_TO_SHOOT: {
                if(myOpMode.gamepad1.yWasPressed()) {
                    sendClostestColorToShooter(ArtifactColor.PURPLE);
                    setState(QUEUEING);
                }  else if(myOpMode.gamepad1.xWasPressed()) {
                    sendClostestColorToShooter(ArtifactColor.GREEN);
                    setState(QUEUEING);
                }  else  if (myOpMode.gamepad1.right_bumper || myOpMode.gamepad1.leftBumperWasPressed()) {
                    fire.setPosition(FIRE_SHOOT);
                    slotColors[currentSlot] = ArtifactColor.UNKNOWN;
                    setState(SHOOTING);
                }
                break;
            }

            case SHOOTING: {
                // wait for the shot to start  !!  THIS PAUSE MAY NOT BE REQUIRED
                if (timeInState(ADVANCE_DELAY_TIME)) {
                    // Move the spindexer to the next ball if there is one
                    if (allArtifactsHeld > 0) {
                        //  advance to the next ball
                        sendClostestColorToShooter(ArtifactColor.ANY);
                        setState(TAKING_SHOT);
                    }
                }
                break;
            }

            case TAKING_SHOT: {
                if (timeInState(FIRE_HOLD_TIME)) {
                    fire.setPosition(FIRE_RETRACT);
                    if (allArtifactsHeld > 0) {
                        setState(QUEUEING);
                    } else {
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
        myOpMode.telemetry.addData("Spin", "%s (s%d) %.1f -> %.1f %s (%.2f)", currentState, currentSlot, currentAngle, targetAngle, inPosition, lastSpindexerServoValue);
        myOpMode.telemetry.addData("Spin colors", "C=%s", currentColor);
        myOpMode.telemetry.addData("Slots", "%s %s %s", slotColors[0], slotColors[1], slotColors[2]);
    }

    /**
     *
     */
   /* private int bestFullSlot(){
        // automatically finds the closest full slot for the shooter
        int bestSlot;

        if (currentSlot == 0){
            if (slotColors[1] != ArtifactColor.UNKNOWN){
                bestSlot = 1;
            } else {
                // must be the best, because all others are empty
                bestSlot = 2;
            }
        } else if (currentSlot == 1){
            if (slotColors[0] != ArtifactColor.UNKNOWN){
                bestSlot = 0;
            } else {
                bestSlot = 2;
            }
        } else {
            if (slotColors[1] != ArtifactColor.UNKNOWN){
                bestSlot = 1;
            } else {
                // must be the best, because all others are empty
                bestSlot = 0;
            }
        }
        return bestSlot;
    } */

    /**
     * sends the beat slot to the intake by deciding on the smallest distance between the three.
     */
    private void sendClostestEmptyToIntake(){
        double closestAngle = 360;
        int    closestSlot  =   -1;
        double destination;

        if (Globals.AXIAL_MOTION >= 0){
            destination = 90;
        } else {
            destination = -90;
        }

        for(int s = 0; s < 3; s++){
            if (slotColors[s] == ArtifactColor.UNKNOWN) {
                double angle = Math.abs(normalizeAngle(destination - currentAngle - HOME_ANGLES[s]));
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

    private void sendClostestColorToShooter(ArtifactColor color){
        double closestAngle = 360;
        int    closestSlot  =  -1;
        double destination  =   0;

        for (int s = 0; s < 3; s++) {
            if ((slotColors[s] == color) || ((color == ArtifactColor.ANY) && (slotColors[s] != ArtifactColor.UNKNOWN))) {
                double angle = Math.abs(normalizeAngle(destination - currentAngle - HOME_ANGLES[s]));
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

    public void sendToShooter(int slot) {
        sendSpindexerTo(SHOOT[slot]);
        currentSlot = slot;
    }

    public void sendToIntake(int slot){
        if (Globals.AXIAL_MOTION >= 0){
            sendSpindexerTo(INTAKE_FRONT[slot]);
        } else {
            sendSpindexerTo(INTAKE_BACK[slot]);
        }
        currentSlot = slot;
    }

    public void sendSpindexerTo(double spindexerAngle) {
        targetAngle = spindexerAngle;
        lastSpindexerServoValue = 0.5 + (targetAngle * PULSE_SCALE_FACTOR);
        spindexer.setPosition(lastSpindexerServoValue);  // +ve angle turns CCW.

        // do we need to clear "InPosition" ?
        if (targetAngle != lastTargetAngle) {
            inPosition = false;
            lastTargetAngle = targetAngle;
        }
    }

    public void queueColor( ArtifactColor colorToQueue) {
        queuedColor = colorToQueue;
    }

    public void resetEncoder(){
        SharedOQ.resetEncoder(OQ_ENCODER_INDEX);
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
}
