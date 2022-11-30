package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name="G-FORCE TELEOP", group = "A Competition")
public class GFORCE_TELEOP extends LinearOpMode {

    boolean weAreRed = false;

    // Joystick constants
    final double AXIAL_RATE = 0.7;
    final double LATERAL_RATE = 0.7;
    final double YAW_RATE = 0.5;
    final double SLOW_TRANSLATE = 0.2;

    GFORCE_KiwiDrive    drive;
    private Elevator    elevator;
    private ConeTracker coneTracker;
    AutoConfig autoConfig = new AutoConfig();

    boolean headingLock = false;
    double  headingSetpoint = 0;
    boolean lastFlip = false;
    boolean flip = false;
    boolean lastStepUp = false;
    boolean lastStepDown = false;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as GFORCE_KiwiDrive's heading controller
    private PIDFController headingController = new PIDFController(new PIDCoefficients(1.25, 0, 0));

    @Override
    public void runOpMode() throws InterruptedException {

        // apply hub performance
        HubPerformance.enable(hardwareMap);
        autoConfig.init(this);
        weAreRed = autoConfig.autoOptions.redAlliance;

        double manualRotate;

        headingController.setInputBounds(0.0, 2.0 * Math.PI);
        headingController.setOutputBounds(-YAW_RATE, YAW_RATE);

        // Initialize robot hardware classes GFORCE_KiwiDrive
        drive = new GFORCE_KiwiDrive(hardwareMap);
        elevator = new Elevator(this, false);
        coneTracker = new ConeTracker(this);

        ElapsedTime cycleTime = new ElapsedTime();

        // Load any prior states from AUTO
        elevator.setState(SharedStates.elevatorState);
        if (SharedStates.currentPose != null) {
            drive.setExternalHeading(SharedStates.currentPose.getHeading());
            SharedStates.currentPose = null;
        } else {
            drive.setExternalHeading(0);
        }

        while (opModeInInit()) {
            telemetry.addData("Alliance", weAreRed ? "RED" : "blue");
            elevator.runStateMachine();
            coneTracker.update();       // testing only
            coneTracker.showRanges();   // testing only
            telemetry.addData("GYRO heading", Math.toDegrees(drive.getExternalHeading()));
            telemetry.update();
        }

        // Lift arm into home position.
        elevator.setLiftTargetPosition(Elevator.ELEVATOR_HOME);
        cycleTime.reset();

        while (opModeIsActive()) {

            // Update everything.
            drive.update();
            elevator.update();
            elevator.runStateMachine();

            telemetry.addData("Elevator", elevator.getStateText());
            elevator.showElevatorState();

            //-----------PILOT-----------
            //check for auto cone tracking

            if (gamepad1.left_bumper && elevator.getWristIsSafe()) {
                elevator.setWristOffset(0);
            }

            // Look for cone tracking request.
            if (gamepad1.left_bumper && elevator.handIsOpen && !elevator.getWristIsSafe() && coneTracker.update() ) {
                coneTracker.showRanges();

                lockNewHeading(drive.getExternalHeading());
                elevator.driverAutoGrabRequest = (!gamepad2.cross && coneTracker.trackGrab());
                drive.setWeightedDrivePower(new Pose2d(coneTracker.trackDrive(), coneTracker.trackStrafe(), coneTracker.trackTurn()));

            } else  {
                elevator.driverAutoGrabRequest = false;

                // Read pose and use it to convery joystick inputs to Field Centric.
                Pose2d poseEstimate = drive.getPoseEstimate();

                Vector2d joysticInput;

                if (gamepad1.dpad_up) {
                    joysticInput = new Vector2d(SLOW_TRANSLATE, 0);
                } else if (gamepad1.dpad_down) {
                    joysticInput = new Vector2d(-SLOW_TRANSLATE,0 );
                } else {
                    // Create a vector from the gamepad x/y inputs
                    // Then, rotate that vector by the inverse of the heading
                    joysticInput = new Vector2d(
                            -gamepad1.left_stick_y * LATERAL_RATE,
                            -gamepad1.right_stick_x * AXIAL_RATE
                    ).rotated(-poseEstimate.getHeading());
                }

                // Determine the rotate rate being requested by pilot.
                manualRotate = (gamepad1.left_trigger - gamepad1.right_trigger) * YAW_RATE;

                // also check to see if the pilot is requesting a spin to one of the XY axes
                if (gamepad1.triangle) {
                    lockNewHeading(Math.toRadians(0));
                } else if (gamepad1.circle) {
                    lockNewHeading(Math.toRadians(270));
                } else if (gamepad1.cross) {
                    lockNewHeading(Math.toRadians(180));
                } else if (gamepad1.square) {
                    lockNewHeading(Math.toRadians(90));
                }

                // are we turning or should heading be locked.
                drive.notTurning();
                if (Math.abs(manualRotate) < 0.01) {
                    if (!headingLock && drive.notTurning()) {
                        lockNewHeading(drive.getExternalHeading());
                    }
                } else {
                    headingLock = false;
                }

                if (headingLock) {
                    // Set desired angular velocity to the heading-controller output
                    double autoRotate = headingController.update(drive.getExternalHeading())  ;

                    telemetry.addData("Auto Powers", "%.2f, %.2f, %.2f", joysticInput.getX(),joysticInput.getY(), autoRotate);

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    joysticInput.getX(),
                                    joysticInput.getY(),
                                    autoRotate
                            )
                    );
                } else {
                    // Pass in the rotated input + right stick value for rotation

                    telemetry.addData("Manual Powers", "%.2f, %.2f, %.2f", joysticInput.getX(),joysticInput.getY(), manualRotate);
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    joysticInput.getX(),
                                    joysticInput.getY(),
                                    manualRotate
                            )
                    );
                }
            }
            // reset heading if double button press
            if (gamepad1.back && gamepad1.start) {
                drive.setPoseEstimate( new Pose2d() );
                drive.setExternalHeading(0);
                headingSetpoint = 0;
                headingController.setTargetPosition(headingSetpoint);
            }

            //-----------CO-PILOT--------------

            // Elevator lift position:
            boolean stepUp = gamepad2.dpad_up;
            boolean stepDown = gamepad2.dpad_down;
            boolean newPosition = false;

            elevator.driverManualGrabRequest = gamepad2.square;

            if (stepUp && !lastStepUp) {
                elevator.levelUp();
            }
            if (stepDown && !lastStepDown) {
                elevator.levelDown();
            }

            lastStepUp   = stepUp;
            lastStepDown = stepDown;

            // Put the wrist in safe position
            flip = (gamepad1.right_bumper || gamepad2.right_bumper);
            if (flip && !lastFlip) {
                if (elevator.getWristIsSafe()) {
                   elevator.setWristOffset(0);
                } else {
                    elevator.setWristOffset(elevator.WRIST_SAFE_OFFSET);
                }
            }
            lastFlip = flip;

            // Manually jog the elevator.
            elevator.jogElevator(-gamepad2.left_stick_y);

            // Display Telemetry data
            telemetry.addData("GYRO heading", Math.toDegrees(drive.getExternalHeading()));
            telemetry.addData("cycle", "%d mS", cycleTime.time(TimeUnit.MILLISECONDS));
            cycleTime.reset();
            telemetry.update();
        }

        // Set Elevator state back to Idle for next autonomous
        elevator.setState(ElevatorState.IDLE);
        SharedStates.currentPose = null;
        SharedStates.elevatorState = ElevatorState.IDLE;
    }

    // Lock in a new heading for the Auto Heading Hold..
    public void lockNewHeading(double heading) {
        headingLock = true;
        headingSetpoint = heading;
        headingController.setTargetPosition(headingSetpoint);
    }

    public void showCurrentPosition() {
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading GYRO.", Math.toDegrees(drive.getExternalHeading()));
        telemetry.update();
    }
}
