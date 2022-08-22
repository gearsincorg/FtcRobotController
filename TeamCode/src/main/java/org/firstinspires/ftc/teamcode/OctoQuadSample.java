package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "OctoQuad Sample", group="OctoQuad")
public class OctoQuadSample extends LinearOpMode {

  private OctoQuadBlocks octoquad;
  private DcMotor left_drive;
  private DcMotor right_drive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    octoquad = hardwareMap.get(OctoQuadBlocks.class, "octoquad");
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");

    // Read the Firmware Revision number from an OctoQuad and returns it as text.
    telemetry.addData("OctoQuad Version ", octoquad.version());
    telemetry.update();
    
    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    
    // Reverse the count-direction of an encoder being read by an
    // OctoQuad interface module.Pass the desired channel number
    // to the block. Valid values are 0 to 7. The direction
    // is inverted if the 'reverse' input is set to 'true'.
    octoquad.reverseEncoderDirection(7, true);
    
    waitForStart();
    
    while (opModeIsActive()) {
      telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
      telemetry.addData(">", "Press X to Reset Encoders");
    
      // Use left stick to drive and right stick to turn
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
    
      left_drive.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
      right_drive.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
    
      // Check for X button to reset encoders
      if (gamepad1.x) {
        // Reset the position of all encoders to zero.
        octoquad.resetAllEncoders();
      }
    
      // Read the position of an encoder connected to an OctoQuad.  Pass the desired channel number to the block.  Valid values are 0 to 7.
      telemetry.addData("Left  Pwr / Pos", 
          JavaUtil.formatNumber(left_drive.getPower(), 2) + " / " + octoquad.getPosition(0));
      telemetry.addData("Right Pwr / Pos", 
          JavaUtil.formatNumber(right_drive.getPower(), 2) + " / " + octoquad.getPosition(1));
      telemetry.update();
    }
  }
}
