/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="GFORCE Autonomous", group = "AA")
public class GFORCEAutonomous extends LinearOpMode
{
    ArmSubsystem arm = new ArmSubsystem(this);

    @Override public void runOpMode()
    {
        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        arm.initialize(true);
        arm.autoGrab();

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // swings arm back to push into submersible
            arm.autoGoToBackPosition();

            while (!arm.inPosition()){
                arm.update();
            }

            arm.stop();

            Actions.runBlocking(
                    robot.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(32)
                            .build());

            arm.clipIt();

            while (!arm.isHome()){
                arm.update();
            }

            arm.stop();

        }
    }
}
