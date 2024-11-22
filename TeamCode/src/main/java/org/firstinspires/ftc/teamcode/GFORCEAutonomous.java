/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
    IntakeSubsystem intake = new IntakeSubsystem(this);

    @Override public void runOpMode()
    {
        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(4, -63, Math.PI / 2));
        arm.initialize(true);
        arm.autoGrab();
        intake.initialize(true);


        //build trajectories
        TrajectoryActionBuilder wall2Sub = robot.actionBuilder(new Pose2d(4, -63, Math.PI / 2))
                .lineToY(-29);

        TrajectoryActionBuilder sub2S1 = robot.actionBuilder(new Pose2d(4, -29, Math.PI / 2))
                .setTangent(- Math.PI / 2)
                .splineToConstantHeading(new Vector2d(36, -24), Math.PI / 2);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // swings arm back to push into submersible
            arm.autoGoToBackPosition();

            Actions.runBlocking(
                    new ParallelAction(
                            arm.actionUpdate(),
                            new SequentialAction(
                                    wall2Sub.build(),
                                    arm.actionClipIt(),
                                    arm.actionWaitForHome(),
                                    sub2S1.build()
                            )
                    )
            );




        }
    }
}
