package org.firstinspires.ftc.teamcode;
/*
 * Copyright (c) 2024 DigitalChickenLabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This OpMode illustrates how to use the DigitalChickenLabs OctoQuad to read MaxBotix sonars
 *
 * This OpMode assumes that the OctoQuad is attached to an I2C interface named "octoquad" in the robot configuration.
 *
 * Wiring:
 *  The OctoQuad will be configured to accept Quadrature encoders on the first four channels and Absolute (pulse width) encoders on the last four channels.
 *
 *  The standard 4-pin to 4-pin cable can be used to connect each Driver Motor encoder to the OctoQuad. (channels 0-3)
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * Note: If you prefer, you can move the two support classes from this file, and place them in their own files.
 *       But leaving them in place is simpler for this example.
 *
 * See the sensor's product page: https://www.tindie.com/products/35114/
 */
@TeleOp(name="OctoQuad Sonar", group="OctoQuad")
public class OctoSonar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Connect to the OctoQuad by looking up its name in the hardwareMap.
        OctoQuad octoquad = hardwareMap.get(OctoQuad.class, "octoquad");
        OctoQuad.EncoderDataBlock encoderDataBlock = new OctoQuad.EncoderDataBlock();

        // Clear out all prior settings and encoder data before setting up desired configuration
        // Assume first 4 channels are relative encoders and the next 4 are absolute encoders
        octoquad.resetEverything();
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);

        // Display the OctoQuad firmware revision
        telemetry.addLine("OctoQuad Firmware v" + octoquad.getFirmwareVersion());
        telemetry.addLine("\nPress START to read encoder values");
        telemetry.update();

        waitForStart();

        // Configure the telemetry for optimal display of data.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(100);

        while (opModeIsActive()) {

            octoquad.readAllEncoderData(encoderDataBlock);
            for (int chan = 4 ; chan < 8; chan++) {
                telemetry.addData("Range", "Max%d %7d uS  %.2f Inches", chan, encoderDataBlock.positions[chan],
                        (double)encoderDataBlock.positions[chan] / 147.0 );
            }

            telemetry.update();
        }
    }
}
