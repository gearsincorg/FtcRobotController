package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
//@Disabled
@TeleOp(name="Manual Feedback Tuner", group = "xx")

/**
 * Updated by Team 2818 to allow tuning individual moves (axial or lateral).
 * Set the desires DISTANCE for the move
 * Set TUNE_STRAFE to either true or false depending on which motion you want to tune.
 * Click the X gamepad button to start each forward (left) and back (right) move.
 */
public final class ManualFeedbackTuner extends LinearOpMode {
    public static double  DISTANCE = 48;
    public static boolean TUNE_STRAFE = false;    //  << set to true for Lateral (Y) tuning.

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), this);

        if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
            throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }
        waitForStart();

        while (opModeIsActive()) {

            // Wait for button press, then Move forward or to the left
            waitForXPress();
            if (opModeIsActive()) {
                if (TUNE_STRAFE) {
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(0, 0, 0))
                                    .setTangent(Math.toRadians(90))
                                    .lineToY(DISTANCE)  // move left
                                    .build());
                } else {
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(0, 0, 0))
                                    .lineToX(DISTANCE)  // move forward
                                    .build());
                }
            }

            // Wait for button press, then Move backwards or to the right
            waitForXPress();
            if (opModeIsActive()) {
                if (TUNE_STRAFE) {
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(0, DISTANCE, 0))
                                    .setTangent(Math.toRadians(-90))
                                    .lineToY(0)  // move right
                                    .build());
                } else {
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(DISTANCE, 0, 0))
                                    .lineToX(0)  // move backward
                                    .build());
                }
            }
        }

    }

    private void waitForXPress() {
        telemetry.addLine("Press X or A button to make next move");
        telemetry.update();
        while (opModeIsActive() && !gamepad1.cross) {
        }
    }
}
