/*
 * Copyright (c) 2022 Digital Chicken Labs
 * Permission is herby granted to use this software for the sole purpose
 * of evaluating the OctoQuad engineering sample boards. No warranty of any
 * kind is provided whether express of implied.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode illustrates using the OctoQuad Encoder module
 * The OpMode assumes you have two motors attached (called left_drive and right_drive) with encoders.
 * You can run the motors using the left and right joysticks,   * and see the position and velocity of each motor change.
 */
@TeleOp(name="OctoQuad Calibrate", group="OctoQuad")
public class OctoQuadCalibrate extends LinearOpMode
{
    OctoQuad    octoquad = null;
    boolean     calibrating = false;
    int[]       lowCal  = new int[OctoQuad.ENCODER_LAST + 1];
    int[]       highCal = new int[OctoQuad.ENCODER_LAST + 1];

    @Override
    public void runOpMode()
    {
        octoquad = hardwareMap.get(OctoQuad.class, "octoquad");
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.ALL_PULSE_WIDTH);
        // octoquad.setSingleChannelPulseWidthParams(4, new OctoQuad.ChannelPulseWidthParams(1, 1045));


        // Get string containing firmware version.
        OctoQuad.FirmwareVersion fw = octoquad.getFirmwareVersion();
        octoquad.resetEverything();

        // Display the OctoQuad firmware revision
        telemetry.addLine("OctoQuad Firmware Version: " + fw.toString());
        telemetry.update();

        sleep(1000);

        // Speed up telemetry for a more rapid display (default is 250 mS)
        telemetry.setMsTransmissionInterval(50);

        while (!isStopRequested())
        {
            // switch between calibrating and not.
            if (!calibrating) {
                if (gamepad1.left_bumper) {
                    java.util.Arrays.fill(lowCal, 0);
                    java.util.Arrays.fill(highCal, 0);
                    calibrating = true;
                } else {
                    telemetry.addData(">", "Press Left Bumper to start Calibration");
                }

            } else {
                if (gamepad1.right_bumper) {
                    calibrating = false;
                } else {
                    telemetry.addData(">", "Press Right Bumper to stop Calibration");
                    updateCalibration();
                }
            }
            telemetry.update();
        }
    }

    public void updateCalibration () {
        int[] positions = octoquad.readAllPositions();

        for (int i = 0; i <= OctoQuad.ENCODER_LAST; i++)  {
            int position = positions[i];

            if (position > highCal[i]) {
                highCal[i] = position;
            }

            if ((position < lowCal[i]) || (lowCal[i] == 0)) {
                lowCal[i] = position;
            }

            int spread = highCal[i] - lowCal[i];

            if (spread > 10) {
                telemetry.addData("Pos ", "%d = %5d : Range = %5d - %5d  (%d)", i, position, lowCal[i], highCal[i], spread);
            } else {
                telemetry.addData("Pos ", "%d %5d :", i, position);
            }
        }
    }
}
