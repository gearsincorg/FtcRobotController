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
        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(4, -63, Math.toRadians(90)));
        arm.initialize(true);
        arm.autoGrab();
        intake.initialize(true);


        //build trajectories
        TrajectoryActionBuilder wallToSub = robot.actionBuilder(new Pose2d(4, -63, Math.toRadians(90)))
                .lineToY(-29);

        TrajectoryActionBuilder subToS1 = robot.actionBuilder(new Pose2d(4, -29, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(35, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(46, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(46, -50), Math.toRadians(-90))
                ;

        TrajectoryActionBuilder s1ToS2 = robot.actionBuilder(new Pose2d(46, -50, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(46, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(52, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(58, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(58, -50), Math.toRadians(-90))
                ;

        TrajectoryActionBuilder s2ToS3 = robot.actionBuilder(new Pose2d(58, -50, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(58, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(68, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(68, -50), Math.toRadians(-90))
                ;

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
                                    wallToSub.build(),
                                    arm.actionClipIt(),
                                    arm.actionWaitForHome(),
                                    subToS1.build(),
                                    s1ToS2.build(),
                                    s2ToS3.build()
                            )
                    )
            );




        }
    }
}
