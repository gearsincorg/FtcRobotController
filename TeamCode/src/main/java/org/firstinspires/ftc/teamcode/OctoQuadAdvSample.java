/*
 * Copyright (c) 2022 Digital Chicken Labs
 * Permission is herby granted to use this software for the sole purpose
 * of evaluating the OctoQuad engineering sample boards. No warranty of any
 * kind is provided whether express of implied.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

/**
 * This OpMode illustrates using the OctoQuad Encoder module
 * The OpMode assumes you have two motors attached (called left_drive and right_drive) with encoders.
 * You can run the motors using the left and right joysticks,   * and see the position and velocity of each motor change.
 */
@TeleOp(name="OctoQuad Advanced Sample", group="OctoQuad")
public class OctoQuadAdvSample extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        OctoQuad octoquad = hardwareMap.get(OctoQuad.class, "octoquad");
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH) ;   

        OctoQuad.ChannelPulseWidthParams params = new OctoQuad.ChannelPulseWidthParams();
        params.min_length_us = 1;
        params.max_length_us = 1045;
        octoquad.setSingleChannelPulseWidthParams(4, params);

        DcMotor leftDrive = hardwareMap.dcMotor.get("left_drive");
        DcMotor rightDrive = hardwareMap.dcMotor.get("right_drive");

        // Get string containing firmware version.
        OctoQuad.FirmwareVersion fw = octoquad.getFirmwareVersion();


        // We want to switch the OctoQuad's I2C port to a faster data rate for less latency, so we need to get access to the Expansion Hub it's tied to
        // If you are using a Control Hub, the default name is "Control Hub", otherwise look at your Robot Configuration for the Expansion Hub name.
        LynxModule module = hardwareMap.get(LynxModule.class, "Control Hub");

        // We also need to designate which port we speed up.  Make sure this matches the port that your OctoQuad is connected to.
        int I2C_BUS = 2;

        LynxI2cConfigureChannelCommand cmd = new LynxI2cConfigureChannelCommand(module, I2C_BUS, LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K);
        try {
            cmd.send();
        } catch (LynxNackException e) {
            e.printStackTrace();
        }

        // Display the OctoQuad firmware revision
        telemetry.addLine("OctoQuad FW v" + fw.toString());
        telemetry.update();
        waitForStart();

        // Speed up telemetry for a more rapid display (default is 250 mS)
        telemetry.setMsTransmissionInterval(50);

        // Prepare an object to hold an entire OctoQuad encoder readable register bank
        OctoQuad.EncoderDataBlock encoderDataBlock = new OctoQuad.EncoderDataBlock();

        // Run stats to determine cycle times.
        MovingStatistics avgTime = new MovingStatistics(100);
        ElapsedTime elapsedTime = new ElapsedTime();

        // Clear out all prior settings and encoder data before setting up desired configuration
        octoquad.resetEverything();

        while (opModeIsActive())
        {
            // Play with the encoder direction.  Invert Motor AND encoder using Dpad.
            telemetry.addData(">", "DPad Up/Dn to rev. Mot & Enc 0.  X to zero all");

            if(gamepad1.dpad_up) {
                octoquad.setSingleEncoderDirection(0, false);
                leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            } else if(gamepad1.dpad_down) {
                octoquad.setSingleEncoderDirection(0, true);
                leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            } else if(gamepad1.x)
            {
                octoquad.resetAllPositions();
            }

            // Run the two motors based on left and right joysticks
            leftDrive.setPower(-gamepad1.left_stick_y);
            rightDrive.setPower(-gamepad1.right_stick_y);

            // Retrieve a full block of OctoQuad register data.  Get everything in one hit for max speed!
            octoquad.readAllEncoderData(encoderDataBlock);

            // Update cycle timestats
            avgTime.add(elapsedTime.nanoseconds());
            elapsedTime.reset();

            // Display position and velocity values using register identifiers.
            for (int p = 0; p < 8; p++) {
                telemetry.addData("Pos", "%d %d", p, encoderDataBlock.positions[p]);
            }
            for (int v = 0; v < 8; v++) {
                telemetry.addData("Vel", "%d %d", v, encoderDataBlock.velocities[v]);
            }
            telemetry.addData("Loop time uS", Math.round(avgTime.getMean()/1000));
            telemetry.update();
        }
    }
}
