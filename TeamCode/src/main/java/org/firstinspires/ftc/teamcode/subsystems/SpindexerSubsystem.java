package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;


public class SpindexerSubsystem {

    private boolean showTelemetry = false;
    private boolean enabled = false;
    private LinearOpMode myOpMode;

    private DcMotor shooter;
    private Servo engage;
    private NormalizedColorSensor color;
    private DigitalChannel magnet;
    private final double COUNTS_PER_REVOLUTION = 537.5;
    private final double MIN_SATURATION = 0.05;
    private final double GREEN_MIN = 120.0;
    private final double GREEN_MAX = 160.0;
    private final double PURPLE_MIN = 230.0;
    private final double PURPLE_MAX = 300.0;

    private int spindexerAngle = 0;
    private ArtifactColor currentColor = ArtifactColor.UNKNOWN;
    private boolean isHomed = false;
    private int currentSlot = 0;
    private int currentSegment = 0;
    private final float[] hsvValues = new float[3];
    private final ArtifactColor[] slotColors = new ArtifactColor[3];

    public SpindexerSubsystem(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize the Subsystem by creating hardware devices.
     * @param showTelemetry
     */
    public void init(boolean showTelemetry) {
        this.showTelemetry = showTelemetry;
        this.enabled = true;

        shooter = myOpMode.hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        engage = myOpMode.hardwareMap.get(Servo.class, "engage");
        engage.setPosition(0.0);

        color = myOpMode.hardwareMap.get(ColorRangeSensor.class, "color");
        color.setGain(10);

        magnet = myOpMode.hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);

        spindexerZero();

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        // skip if not initialized
        if (!enabled) return;

        currentColor = ArtifactColor.UNKNOWN;

        spindexerAngle = (int)(shooter.getCurrentPosition() / COUNTS_PER_REVOLUTION * 360) % 360;
        currentSlot = spindexerAngle / 120;
        currentSegment = spindexerAngle / 24;


        NormalizedRGBA colors = color.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        //checking the hue and saturation of the color sensor
        //saturation needs to be high enough use the hue value
        //find which range the hue resides in to decide the color
        if (hsvValues[1] > MIN_SATURATION){
            if ((hsvValues[0] > GREEN_MIN) && (hsvValues[0] < GREEN_MAX)){
                currentColor = ArtifactColor.GREEN;
            } else if ((hsvValues[0] > PURPLE_MIN) && (hsvValues[0] < PURPLE_MAX)){
                currentColor = ArtifactColor.PURPLE;
            }
        }

        //only update slot colors when in range of color sensor
        if ((currentSegment == 1) || (currentSegment == 6) || (currentSegment == 11)) {
            slotColors[currentSlot] = currentColor;
        }

        if (showTelemetry) {
            myOpMode.telemetry.addData("Spindexer Angle", spindexerAngle);

            myOpMode.telemetry.addLine()
                    .addData("Slot 0", "%s", slotColors[0])
                    .addData("Slot 1", "%s", slotColors[1])
                    .addData("Slot 2", "%s", slotColors[2]);

            myOpMode.telemetry.addData("Current color", currentColor);

            myOpMode.telemetry.addData("magnet", magnet.getState());

            myOpMode.telemetry.addData("Current Segment", currentSegment);

            myOpMode.telemetry.addData("Current Slot", currentSlot);
        }

        if (myOpMode.gamepad1.dpad_up) {
            engage.setPosition(1.0);
        } else {
            engage.setPosition(0.0);
        }
    }

    //Homes the Spindexer during init
    public void spindexerZero(){
        // skip if not initialized
        if (!enabled) return;

        isHomed = false;
        shooter.setPower(0.1);

        // wait until it detects the magnet
        while (myOpMode.opModeInInit() && magnet.getState()){
            myOpMode.telemetry.addData("magnet", magnet.getState());
            myOpMode.telemetry.update();
        }

        shooter.setPower(0.0);
        isHomed = true;
    }

    public void rotate(){
        // skip if not initialized
        if (!enabled) return;

        shooter.setPower(0.15);
    }

    //-------------------------------------------------------------------------
    // ACTION  methods
    //-------------------------------------------------------------------------

    public Action actionUpdate(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                update();
                return true;
            }
        };
    }

}
