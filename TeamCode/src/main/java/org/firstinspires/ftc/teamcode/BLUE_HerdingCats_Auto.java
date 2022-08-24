/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have gyro or IMU otherwise you would use: RobotAutoDriveByEncoder;
 *  This sample program assumes that you have a BOSCH BNO055 Inertial Measurement Unit (IMU)
 *  This IMU is found in REV Expansion Hubs and Control Hubs produced prior to the 2023 FTC season.
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.  So please
 *  test your motors and set the correct direction (see runOpMode())
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the movement profile
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set when resetHeading() is called.
 *  This is done when the Play button is pressed on the Driver station.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="BLUE Autonomous BLUE", group="Competition", preselectTeleOp="Herding Cats TELEOP")
//@Disable
public class BLUE_HerdingCats_Auto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor     leftDrive   = null;
    private DcMotor     rightDrive  = null;
    private DcMotor     grabber     = null;
    private DcMotor     lifter      = null;
    private BNO055IMU   imu         = null;      // Control Hub IMU

    private double          robotHeading  = 0;
    private double          headingOffset = 0;


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    // static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     COUNTS_PER_MOTOR_REV    = 526 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // Used to calculate circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.2;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro

    // Define the Proportional control coefficient for "heading control".
    // We define one value when rotating (larger errors), and the other is used when driving straight (smaller errors).
    static final double     P_TURN_COEFF            = 0.04;     // greater value is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.03;     // greater value is more responsive, but also less stable

    //  Manipulator controls
    final int    NOT_MOVING = 5;
    final int    LIFTER_STANDBY   =  930 ;
    final int    GRABBER_OPEN     =  120 ;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftDrive   = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive  = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabber     = hardwareMap.get(DcMotor.class, "grabber");
        grabber.setDirection(DcMotor.Direction.FORWARD);
        grabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter      = hardwareMap.get(DcMotor.class, "lifter");
        lifter.setDirection(DcMotor.Direction.REVERSE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Ensure the robot it stationary
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Home the collector
        if (!LifterStatus.lifterHomed) {
            homeCollector();
        }

        telemetry.addData(">", "BLUE Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value)
        while (opModeInInit()) {
            telemetry.addData(">", "! BLUE ! Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        // Reset the encoders for distance tracking. and zero the robot heading.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroDrive(DRIVE_SPEED, 102, 0);
        gyroHold(TURN_SPEED, 0,1);
        gyroTurn(TURN_SPEED, 90);
        gyroHold(TURN_SPEED,90, 1);
        gyroDrive(TURN_SPEED,18,90);

        telemetry.addData("Path", "Complete");
        telemetry.addData("Final Heading", "%5.0f", getHeading());
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

   /**
    *  Method to drive on a fixed compass heading (angle), for the requested number of Inches.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for +/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param angle      Absolute Angle (in Degrees) relative to last resetHeading() call.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from robotHeading.
    */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (leftDrive.isBusy() && rightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftDrive.setPower(leftSpeed);
                rightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", angle, robotHeading);
                telemetry.addData("Error:Steer",  "%5.1f:%5.1f",  error, steer);
                telemetry.addData("Target L:R",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual L:R",  "%7d:%7d",      leftDrive.getCurrentPosition(),
                                                             rightDrive.getCurrentPosition());
                telemetry.addData("Speed L:R",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double   leftSpeed;
        double   rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target/Current", "%5.2f / %5.0f", angle, robotHeading);
        telemetry.addData("Error/Steer", "%5.2f / %5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        // calculate error in -179 to +180 range  (
        return (normalizeHeading(targetAngle - getHeading()));
    }

    /**
     * read and save the current robot heading
     */
    public double getHeading() {
        robotHeading = normalizeHeading(getRawHeading() - headingOffset);
        return robotHeading;
    }

    /**
     * Return a normalized heading.  This means, convert it to a +/- 180 degree value
     */
    public double normalizeHeading(double nHeading) {
        while (nHeading > 180)  nHeading -= 360;
        while (nHeading <= -180) nHeading += 360;
        return (nHeading);
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the heading back to zero
     */
    public void resetHeading() {
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void homeCollector() {

        telemetry.addData(">", "Homing Collector.");    //
        telemetry.update();

        grabber.setPower(0.0);
        lifter.setPower(0.0);
        grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int grabberPos = grabber.getCurrentPosition();
        int lifterPos  = lifter.getCurrentPosition();
        int lastGrabberPos = grabberPos;
        int lastLifterPos  = lifterPos;

        // First close the grabber.
        grabber.setPower(-0.2);
        sleep(200);
        while (!isStopRequested()){
            grabberPos = grabber.getCurrentPosition();
            int dif = Math.abs(grabberPos - lastGrabberPos);
            telemetry.addData("Grabber", "Dif = %d", dif);
            telemetry.update();
            if (dif < NOT_MOVING) {
                // Stop motor and reset encoder.
                grabber.setPower(0.0);
                grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            } else {
                sleep(100);
            }
            lastGrabberPos = grabberPos;
        }
        grabber.setPower(0.0);

        // Now lower the lifter to the ground.
        lifter.setPower(-0.10);
        sleep(200);
        while (!isStopRequested()){
            lifterPos = lifter.getCurrentPosition();
            int dif = Math.abs(lifterPos - lastLifterPos);
            telemetry.addData("Lifter", "Dif = %d", dif);
            telemetry.update();
            if (Math.abs(lifterPos - lastLifterPos) < NOT_MOVING) {
                // Stop motor and reset encoder.
                lifter.setPower(0.0);
                lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            } else {
                sleep(100);
            }
            lastLifterPos = lifterPos;
        }

        // Move to standby position.
        lifter.setPower(0.0);
        lifter.setTargetPosition(LIFTER_STANDBY);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.7);

        grabber.setPower(0.0);
        grabber.setTargetPosition(GRABBER_OPEN);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber.setPower(0.5);

        LifterStatus.lifterHomed = true;

        telemetry.addData("Collector", "STANDBY");
        telemetry.update();
    }

}
