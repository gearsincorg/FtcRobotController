/*
 * Copyright (c) 2022 Digital Chicken Labs
 * Permission is herby granted to use this software for the sole purpose
 * of evaluating the OctoQuad engineering sample boards. No warranty of any
 * kind is provided whether express of implied.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

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
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    
    // Reverse the count-direction of an encoder being read by an
    // OctoQuad interface module.Pass the desired channel number
    // to the block. Valid values are 0 to 7. The direction
    // is inverted if the 'reverse' input is set to 'true'.
    octoquad.reverseEncoderDirection(0, true);
    
    waitForStart();
    
    while (opModeIsActive()) {
      
      telemetry.addData(">", "Press X to Reset Encoders");
      
      // Use left stick to drive and right stick to turn
      left_drive.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
      right_drive.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
      
      // Check for X button to reset encoders
      if (gamepad1.x) {
        // Reset the position of all encoders to zero.
        octoquad.resetAllEncoders();
      }
      telemetry.addData("Left Power", JavaUtil.formatNumber(left_drive.getPower(), 2));
      
      // Read the position of encoder 0 connected to an OctoQuad.
      telemetry.addData("Left Position ", octoquad.getPosition(0));
      
      telemetry.addData("Right Power", JavaUtil.formatNumber(right_drive.getPower(), 2));
      
      // Read the position of encoder 1 connected to an OctoQuad.
      telemetry.addData("Right Position", octoquad.getPosition(1));
      telemetry.update();
    }
  }
}
