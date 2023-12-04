package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Alert {

    private final LinearOpMode myOpMode;
    private boolean showTelemetry = false;

    private final ElapsedTime alertTimer = new ElapsedTime();
    private AlertState  alertState = AlertState.OFF;
    private boolean onCycle = false;
    private double cycleHalfPeriod = 0.5;

    private DigitalChannel leftGreenLED;
    private DigitalChannel leftRedLED;
    private DigitalChannel rightGreenLED;
    private DigitalChannel rightRedLED;

    // Alert Constructor
    public Alert(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void initialize(boolean showTelemetry) {

        this.showTelemetry = showTelemetry;
        leftRedLED = myOpMode.hardwareMap.get(DigitalChannel.class, "led0");
        leftGreenLED = myOpMode.hardwareMap.get(DigitalChannel.class, "led1");
        rightRedLED = myOpMode.hardwareMap.get(DigitalChannel.class, "led2");
        rightGreenLED = myOpMode.hardwareMap.get(DigitalChannel.class, "led3");

        leftRedLED.setMode(DigitalChannel.Mode.OUTPUT);
        leftGreenLED.setMode(DigitalChannel.Mode.OUTPUT);
        rightRedLED.setMode(DigitalChannel.Mode.OUTPUT);
        rightGreenLED.setMode(DigitalChannel.Mode.OUTPUT);

        update();
        setLeftLED(LEDcolor.OFF);
        setRightLED(LEDcolor.OFF);
    }

    public void update() {
        if (alertTimer.time() >= cycleHalfPeriod){
            alertTimer.reset();
            onCycle = !onCycle;

            if (showTelemetry) {
                myOpMode.telemetry.addData("alert", alertState)  ;
            }

            // flip the LED state based on current mode
            switch (alertState) {
                case OFF:
                    setLeftLED(LEDcolor.OFF);
                    setRightLED(LEDcolor.OFF);
                    break;

                case VIDEO_ERROR:
                    if (onCycle) {
                        setLeftLED(LEDcolor.RED);
                        setRightLED(LEDcolor.GREEN);
                    } else {
                        setLeftLED(LEDcolor.GREEN);
                        setRightLED(LEDcolor.RED);
                    }
                    break;

                case AUTO_PIXEL:
                    if (onCycle) {
                        if (Globals.PURPLE_PIXEL_ON_RIGHT) {
                            setRightLED(LEDcolor.RED);
                        } else {
                            setLeftLED(LEDcolor.RED);
                        }
                    } else {
                        setLeftLED(LEDcolor.OFF);
                        setRightLED(LEDcolor.OFF);
                    }
                    break;

                case TELEOP_GRABBER:
                    if (Globals.LEFT_GRABBER_CLOSED) {
                        if (onCycle) {
                            setLeftLED(LEDcolor.RED);
                        } else {
                            setLeftLED(LEDcolor.OFF);
                        }
                    } else {
                        setLeftLED(LEDcolor.GREEN);
                    }

                    if (Globals.RIGHT_GRABBER_CLOSED) {
                        if (onCycle) {
                            setRightLED(LEDcolor.RED);
                        } else {
                            setRightLED(LEDcolor.OFF);
                        }
                    } else {
                        setRightLED(LEDcolor.GREEN);
                    }
                    break;
            }
        }
    }

    public void setState(AlertState newState) {
        if (newState != alertState) {
            alertState = newState;
            alertTimer.reset();
            onCycle = false;
            switch (alertState) {
                case OFF:
                    cycleHalfPeriod = 1.0;
                    break;
                case VIDEO_ERROR:
                    cycleHalfPeriod = 0.2;
                    break;
                case AUTO_PIXEL:
                    cycleHalfPeriod = 0.4;
                    break;
                case TELEOP_GRABBER:
                    cycleHalfPeriod = 0.1;
                    break;
            }
            update();
        }
    }


    private void setLeftLED(LEDcolor color) {
        switch (color) {
            case OFF:
                leftGreenLED.setState(true);
                leftRedLED.setState(true);
                break;

            case RED:
                leftGreenLED.setState(true);
                leftRedLED.setState(false);
                break;

            case GREEN:
                leftGreenLED.setState(false);
                leftRedLED.setState(true);
                break;

            case ORANGE:
                leftGreenLED.setState(false);
                leftRedLED.setState(false);
                break;
        }
    }

    private void setRightLED(LEDcolor color) {
        switch (color) {
            case OFF:
                rightGreenLED.setState(true);
                rightRedLED.setState(true);
                break;

            case RED:
                rightGreenLED.setState(true);
                rightRedLED.setState(false);
                break;

            case GREEN:
                rightGreenLED.setState(false);
                rightRedLED.setState(true);
                break;

            case ORANGE:
                rightGreenLED.setState(false);
                rightRedLED.setState(false);
                break;
        }
    }

}

enum LEDcolor {
    OFF,
    RED,
    GREEN,
    ORANGE
}