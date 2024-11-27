/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="GFORCE Autonomous", group = "AA")
public class GFORCEAutonomous extends LinearOpMode
{
    MecanumDrive    robot;
    OctoQuadIF      octoQuad = new OctoQuadIF(this);
    ArmSubsystem    arm      = new ArmSubsystem(this);
    IntakeSubsystem intake   = new IntakeSubsystem(this);
    VisionSubsystem blob     = new VisionSubsystem(this);

        @Override public void runOpMode()
    {
        robot = new MecanumDrive(hardwareMap, new Pose2d(4, -63, Math.toRadians(90)));
        octoQuad.initialize(true);
        arm.initialize(true);
        intake.initialize(true);
        blob.initilaize(true);

        telemetry.setMsTransmissionInterval(25);

        //build trajectories
        TrajectoryActionBuilder wallToSub = robot.actionBuilder(new Pose2d(4, -63, Math.toRadians(90)))
                .lineToY(-30);

        TrajectoryActionBuilder subToAllSamples = robot.actionBuilder(new Pose2d(4, -30, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(44, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(44, -47), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(44, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(53, -24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(53, -47), Math.toRadians(90))

                //.splineToConstantHeading(new Vector2d(53, -24), Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(58, -12), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(62, -24), Math.toRadians(-90))
                //.splineToConstantHeading(new Vector2d(62, -45), Math.toRadians(90))
                ;

        TrajectoryActionBuilder samplesToSpecimen = robot.actionBuilder(new Pose2d(53, -47, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(30, -52), Math.toRadians(-180))
                ;

        TrajectoryActionBuilder specimenToSub2 = robot.actionBuilder(new Pose2d(30, -63, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(12, -48), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -30), Math.toRadians(90))
                ;

        TrajectoryActionBuilder specimenToSub3 = robot.actionBuilder(new Pose2d(30, -63, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(12, -48), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-4, -30), Math.toRadians(90))
                ;

        TrajectoryActionBuilder subToSpecimen = robot.actionBuilder(new Pose2d(0, -30, Math.toRadians(90)))
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(30, -52), Math.toRadians(0))
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
                                    arm.actionWaitForState(ArmStates.LOWERING),
                                    subToAllSamples.build(),
                                    samplesToSpecimen.build(),
                                    actionDriveToSpecimen(),
                                    arm.actionSetState(ArmStates.GRABBING),
                                    arm.actionWaitForState(ArmStates.GRABBED),
                                    specimenToSub2.build(),
                                    arm.actionClipIt(),
                                    arm.actionWaitForState(ArmStates.LOWERING),
                                    subToSpecimen.build(),
                                    actionDriveToSpecimen(),
                                    arm.actionSetState(ArmStates.GRABBING),
                                    arm.actionWaitForState(ArmStates.GRABBED),
                                    specimenToSub3.build(),
                                    arm.actionClipIt(),
                                    arm.actionWaitForState(ArmStates.LOWERING)

                            )
                    )
            );
        }
    }

    final double WITHIN_RANGE   = 2.00 ;
    final double APPROACH_SPEED = -0.3 ;
    final double CLICK_ON_SPEED = -0.2 ;

    final double STRAFE_GAIN = 0.7 ;
    final double MAX_TURN_POWER = 0.4 ;
    final double YAW_GAIN_RAD = 0.02 * 180 / Math.PI;    // Strength of Yaw position control

    public Action actionDriveToSpecimen(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                octoQuad.update();
                double yError = octoQuad.getBackRangeInches();
                robot.updatePoseEstimate();

                // try to get within range of specimen
                if ((Math.abs(yError) > WITHIN_RANGE) || (yError == 0)) {
                    double strafe = 0;
                    double drive = 0;
                    double xError = 0;

                    // strafe to be in front of specimen
                    ColorTarget target = blob.getTarget();
                    if (target.valid) {
                        xError = target.x;
                        strafe = xError * STRAFE_GAIN;
                    }

                    // slow down when you get close to wall.
                    if (yError >= 5) {
                        drive = APPROACH_SPEED;
                    } else {
                        drive = CLICK_ON_SPEED;
                    }

                    // try to stay square to wall.
                    double yawError = (Math.PI / 2) - robot.pose.heading.toDouble() ;
                    double yaw = MathUtils.clamp(yawError * YAW_GAIN_RAD  , -MAX_TURN_POWER, MAX_TURN_POWER);

                    telemetry.update();

                    // send drive power to wheels, and continue action
                    robot.setDrivePowers(new PoseVelocity2d(new Vector2d(drive, strafe), 0));
                    return true;
                } else {

                    telemetry.update();

                    // stop moving and exit action.
                    robot.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
                    return false;
                }
            }
        };
    }

}
