package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;


public class SpindexerSubsystem {

    private boolean showTelemetry;
    LinearOpMode myOpmode;
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
        myOpmode = opmode;
    }

    /**
     * Initialize the Subsystem by creating hardware devices.
     * @param showTelemetry
     */
    public void init(boolean showTelemetry) {
        this.showTelemetry = showTelemetry;

        shooter = myOpmode.hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        engage = myOpmode.hardwareMap.get(Servo.class, "engage");
        engage.setPosition(0.0);

        color = myOpmode.hardwareMap.get(ColorRangeSensor.class, "color");
        color.setGain(10);

        magnet = myOpmode.hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);

        spindexerZero();

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
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
            myOpmode.telemetry.addData("Spindexer Angle", spindexerAngle);

            myOpmode.telemetry.addLine()
                    .addData("Slot 0", "%s", slotColors[0])
                    .addData("Slot 1", "%s", slotColors[1])
                    .addData("Slot 2", "%s", slotColors[2]);

            myOpmode.telemetry.addData("Current color", currentColor);

            myOpmode.telemetry.addData("magnet", magnet.getState());

            myOpmode.telemetry.addData("Current Segment", currentSegment);

            myOpmode.telemetry.addData("Current Slot", currentSlot);
        }

        if (myOpmode.gamepad1.dpad_up) {
            engage.setPosition(1.0);
        } else {
            engage.setPosition(0.0);
        }
    }

    //Homes the Spindexer during init
    public void spindexerZero(){
        isHomed = false;
        shooter.setPower(0.1);

        // wait until it detects the magnet
        while (myOpmode.opModeInInit() && magnet.getState()){
            myOpmode.telemetry.addData("magnet", magnet.getState());
            myOpmode.telemetry.update();
        }

        shooter.setPower(0.0);
        isHomed = true;
    }

    public void rotate(){
        shooter.setPower(0.15);
    }
}
