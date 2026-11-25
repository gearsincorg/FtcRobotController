package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * OctoQuad CRC hunt (2026-11-25): running a drive motor made OctoQuad I2C reads fail (the cable was
 * strapped next to the front-left motor).  This reads the OctoQuad exactly as Pedro does (an
 * OctoQuadLocalizer from Constants, one readLocalizerData() per loop) and counts the bad reads
 * while a drive motor runs.
 *
 * PUT THE ROBOT ON BLOCKS.  Hold a button to run that motor at POWER (gamepad 1, laid out like the robot):
 *   Triangle front left     Circle front right
 *   Square   back left      Cross  back right
 * Pressing a button clears that motor's count; its line updates while held and stays when released.
 */
@Utility(name = "OctoQuad Bus Test", description = "Test")
public class OctoQuadBusTestOpMode extends LinearOpMode {

    private static final double POWER = 0.5;

    private static final String[] NAMES = { "front_left_drive", "front_right_drive", "back_left_drive", "back_right_drive" };
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final int[] loops = new int[4], bad = new int[4];
    private final boolean[] held = new boolean[4];

    @Override
    public void runOpMode() {
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i]);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motors[i].setPower(0);
        }

        telemetry.addLine("Setting up the OctoQuad as Pedro does (resets it and waits for the IMU)...");
        telemetry.update();
        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, Constants.localizerConfig());
        OctoQuad octoQuad = localizer.octoQuad;
        OctoQuad.LocalizerDataBlock block = new OctoQuad.LocalizerDataBlock();

        telemetry.addLine("ROBOT ON BLOCKS.  Hold Triangle FL, Circle FR, Square BL, Cross BR.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // The OctoQuad read, exactly as OctoQuadLocalizer.update() does it
            octoQuad.readLocalizerData(block);
            boolean ok = block.crcOk;

            boolean[] now = { gamepad1.triangle, gamepad1.circle, gamepad1.square, gamepad1.cross };
            for (int i = 0; i < 4; i++) {
                if (now[i] && !held[i]) { loops[i] = 0; bad[i] = 0; }   // pressed: new count
                if (now[i]) { loops[i]++; if (!ok) bad[i]++; }
                if (now[i] != held[i]) motors[i].setPower(now[i] ? POWER : 0);
                held[i] = now[i];
            }

            telemetry.addLine("Hold Triangle FL, Circle FR, Square BL, Cross BR  (power " + POWER + ")");
            for (int i = 0; i < 4; i++) {
                double pct = loops[i] > 0 ? 100.0 * bad[i] / loops[i] : 0;
                telemetry.addLine(String.format("%-17s %4d/%-5d bad (%5.2f%%)%s",
                        NAMES[i], bad[i], loops[i], pct, held[i] ? "  RUNNING" : ""));
            }
            telemetry.update();
        }

        for (DcMotorEx m : motors) m.setPower(0);
    }
}
