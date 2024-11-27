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

        //build trajectories
        TrajectoryActionBuilder wallToSub = robot.actionBuilder(new Pose2d(4, -63, Math.toRadians(90)))
                .lineToY(-30);

        TrajectoryActionBuilder subToAllSamples = robot.actionBuilder(new Pose2d(4, -30, Math.toRadians(90)))
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

        TrajectoryActionBuilder samplesToSpecimen = robot.actionBuilder(new Pose2d(66, -45, Math.toRadians(90)))
                .setTangent(Math.toRadians(120))
                .splineToConstantHeading(new Vector2d(30, -48), Math.toRadians(-120))
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
                                    subToAllSamples.build(),
                                    samplesToSpecimen.build(),
                                    actionDriveToSpecimen()

                            )
                    )
            );
        }
    }

    final double WITHIN_RANGE   = 0.75 ;
    final double APPROACH_SPEED = 0.3 ;
    final double CLICK_ON_SPEED = 0.2 ;

    final double STRAFE_GAIN = 1.5 ;
    final double MAX_TURN_POWER = 0.4 ;
    final double YAW_GAIN_RAD = 0.02 * 180 / Math.PI;    // Strength of Yaw position control

    public Action actionDriveToSpecimen(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                double yError = octoQuad.getBackRangeInches();

                // try to get within range of specimen
                if (Math.abs(yError) > WITHIN_RANGE) {
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
                    double yawError = (-Math.PI / 2) - robot.pose.heading.toDouble() ;
                    double yaw = MathUtils.clamp(yawError * YAW_GAIN_RAD  , -MAX_TURN_POWER, MAX_TURN_POWER);

                    // send drive power to wheels, and continue action
                    robot.setDrivePowers(new PoseVelocity2d(new Vector2d(drive, strafe), yaw));
                    return true;
                } else {
                    // stop moving and exit action.
                    robot.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
                    return false;
                }
            }
        };
    }

}
