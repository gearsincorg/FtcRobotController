/* Copyright (c) 2017-2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.hardware.rev.RevColorSensorV3;


/*
 * This OpMode shows how to use a color sensor in a generic
 * way, regardless of which particular make or model of color sensor is used. The OpMode
 * assumes that the color sensor is configured with a name of "sensor_color".
 *
 * There will be some variation in the values measured depending on the specific sensor you are using.
 *
 * You can increase the gain (a multiplier to make the sensor report higher values) by holding down
 * the A button on the gamepad, and decrease the gain by holding down the B button on the gamepad.
 *
 * If the color sensor has a light which is controllable from software, you can use the X button on
 * the gamepad to toggle the light on and off. The REV sensors don't support this, but instead have
 * a physical switch on them to turn the light on and off, beginning with REV Color Sensor V2.
 *
 * If the color sensor also supports short-range distance measurements (usually via an infrared
 * proximity sensor), the reported distance will be written to telemetry. As of September 2020,
 * the only color sensors that support this are the ones from REV Robotics. These infrared proximity
 * sensor measurements are only useful at very small distances, and are sensitive to ambient light
 * and surface reflectivity. You should use a different sensor if you need precise distance measurements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Brushland Setup", group = "sample_color")
@Disabled
public class BrushlandSetup extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    ColorRangefinder crf = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "sample_color"));

        /*
        Using this example configuration, you can detect all three sample colors based on which pin is reading true:
        both      --> yellow
        only pin0 --> blue
        only pin1 --> red
        neither   --> no object
         */
    crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 150, 250); // blue
    crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 61, 120); // yellow
    crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 100); // 100mm or closer requirement

    crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0, 60);
    crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 61, 120);
    crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 100); // 100mm or closer requirement

    telemetry.addLine("done");
    telemetry.update();
    waitForStart();

    stop();
  }
}

  /**
   * Helper class for configuring the Brushland Labs Color Rangefinder.
   * Online documentation: <a href="https://docs.brushlandlabs.com">...</a>
   */
  class ColorRangefinder {
    public final RevColorSensorV3 emulator;
    private final I2cDeviceSynchSimple i2c;

    public ColorRangefinder(RevColorSensorV3 emulator) {
      this.emulator = emulator;
      this.i2c = emulator.getDeviceClient();
      this.i2c.enableWriteCoalescing(true);
    }

    /**
     * Configure Pin 0 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin0Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
      setDigital(PinNum.PIN0, digitalMode, lowerBound, higherBound);
    }

    /**
     * Configure Pin 1 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin1Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
      setDigital(PinNum.PIN1, digitalMode, lowerBound, higherBound);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 0's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin0DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
      setPin0Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 1's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin1DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
      setPin1Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin0InvertHue() {
      setPin0DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin1InvertHue() {
      setPin1DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    /**
     * The denominator is what the raw sensor readings will be divided by before being scaled to 12-bit analog.
     * For the full range of that channel, leave the denominator as 65535 for colors or 100 for distance.
     * Smaller values will clip off higher ranges of the data in exchange for higher resolution within a lower range.
     */
    public void setPin0Analog(AnalogMode analogMode, int denominator) {
      byte denom0 = (byte) (denominator & 0xFF);
      byte denom1 = (byte) ((denominator & 0xFF00) >> 8);
      i2c.write(PinNum.PIN0.modeAddress, new byte[]{analogMode.value, denom0, denom1});
    }

    /**
     * Configure Pin 0 as analog output of one of the six data channels.
     * To read analog, make sure the physical switch on the sensor is flipped away from the
     * connector side.
     */
    public void setPin0Analog(AnalogMode analogMode) {
      setPin0Analog(analogMode, analogMode == AnalogMode.DISTANCE ? 100 : 0xFFFF);
    }

    public float[] getCalibration() {
      java.nio.ByteBuffer bytes =
              java.nio.ByteBuffer.wrap(i2c.read(CALIB_A_VAL_0, 16)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
      return new float[]{bytes.getFloat(), bytes.getFloat(), bytes.getFloat(), bytes.getFloat()};
    }

    /**
     * Save a brightness value of the LED to the sensor.
     *
     * @param value brightness between 0-255
     */
    public void setLedBrightness(int value) {
      i2c.write8(LED_BRIGHTNESS, value);
    }

    /**
     * Change the I2C address at which the sensor will be found. The address can be reset to the
     * default of 0x52 by holding the reset button.
     *
     * @param value new I2C address from 1 to 127
     */
    public void setI2cAddress(int value) {
      i2c.write8(I2C_ADDRESS_REG, value << 1);
    }

    /**
     * Read distance via I2C
     * @return distance in millimeters
     */
    public double readDistance() {
      java.nio.ByteBuffer bytes =
              java.nio.ByteBuffer.wrap(i2c.read(PS_DISTANCE_0, 4)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
      return bytes.getFloat();
    }

    private void setDigital(
            PinNum pinNum,
            DigitalMode digitalMode,
            double lowerBound,
            double higherBound
    ) {
      int lo, hi;
      if (lowerBound == higherBound) {
        lo = (int) lowerBound;
        hi = (int) higherBound;
      } else if (digitalMode.value <= DigitalMode.HSV.value) { // color value 0-255
        lo = (int) Math.round(lowerBound / 255.0 * 65535);
        hi = (int) Math.round(higherBound / 255.0 * 65535);
      } else { // distance in mm
        float[] calib = getCalibration();
        if (lowerBound < .5) hi = 2048;
        else hi = rawFromDistance(calib[0], calib[1], calib[2], calib[3], lowerBound);
        lo = rawFromDistance(calib[0], calib[1], calib[2], calib[3], higherBound);
      }

      byte lo0 = (byte) (lo & 0xFF);
      byte lo1 = (byte) ((lo & 0xFF00) >> 8);
      byte hi0 = (byte) (hi & 0xFF);
      byte hi1 = (byte) ((hi & 0xFF00) >> 8);
      i2c.write(pinNum.modeAddress, new byte[]{digitalMode.value, lo0, lo1, hi0, hi1});
      try {
        Thread.sleep(25);
      } catch (InterruptedException e) {
        throw new RuntimeException(e);
      }
    }

    private double root(double n, double v) {
      double val = Math.pow(v, 1.0 / Math.abs(n));
      if (n < 0) val = 1.0 / val;
      return val;
    }

    private int rawFromDistance(float a, float b, float c, float x0, double mm) {
      return (int) (root(b, (mm - c) / a) + x0);
    }

    private enum PinNum {
      PIN0(0x28), PIN1(0x2D);

      private final byte modeAddress;

      PinNum(int modeAddress) {
        this.modeAddress = (byte) modeAddress;
      }
    }

    // other writeable registers
    private static final byte CALIB_A_VAL_0 = 0x32;
    private static final byte PS_DISTANCE_0 = 0x42;
    private static final byte LED_BRIGHTNESS = 0x46;
    private static final byte I2C_ADDRESS_REG = 0x47;

    public static int invertHue(int hue360) {
      return ((hue360 - 180) % 360);
    }

    public enum DigitalMode {
      RED(1), BLUE(2), GREEN(3), ALPHA(4), HSV(5), DISTANCE(6);
      public final byte value;

      DigitalMode(int value) {
        this.value = (byte) value;
      }
    }

    public enum AnalogMode {
      RED(13), BLUE(14), GREEN(15), ALPHA(16), HSV(17), DISTANCE(18);
      public final byte value;

      AnalogMode(int value) {
        this.value = (byte) value;
      }
    }

  }
