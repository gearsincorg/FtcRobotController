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

@Autonomous(name="GFORCE Autonomous", group = "AA")
public class GFORCEAutonomous extends LinearOpMode
{
    OctoQuadIF   octoQuad = new OctoQuadIF(this);
    ArmSubsystem arm = new ArmSubsystem(this);
    IntakeSubsystem intake = new IntakeSubsystem(this);
    VisionSubsystem blob = new VisionSubsystem(this);

    @Override public void runOpMode()
    {
        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(4, -63, Math.toRadians(90)));
        octoQuad.initialize(true);
        arm.initialize(true);
        intake.initialize(true);
        blob.initilaize(true);

        //build trajectories
        TrajectoryActionBuilder wallToSub = robot.actionBuilder(new Pose2d(4, -63, Math.toRadians(90)))
                .lineToY(-29);

        TrajectoryActionBuilder subToS1 = robot.actionBuilder(new Pose2d(4, -29, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(41, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(46, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(46, -45), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(46, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(51, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(56, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(56, -45), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(56, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(61, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(66, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(66, -45), Math.toRadians(90))
                ;

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Score on submersible and then push three samples into obs zone
            Actions.runBlocking(
                    new ParallelAction(
                            arm.actionUpdate(),
                            new SequentialAction(
                                    arm.actionSetState(ArmStates.GRABBED),
                                    wallToSub.build(),
                                    arm.actionClipIt(),
                                    arm.actionWaitForState(ArmStates.READY),
                                    subToS1.build()
                            )
                    )
            );
        }
    }
}
