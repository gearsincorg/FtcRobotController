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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Herding Cats TELEOP", group="Competition")
public class HerdingCats_Teleop extends LinearOpMode {

    final double TURN_RATE  = 0.20;
    final double DRIVE_RATE = 0.45;
    final double TURBO_TURN_RATE  = 0.50;
    final double TURBO_DRIVE_RATE = 1.00;
    final double TURN_CRAWL  = 0.10;
    final double DRIVE_CRAWL = 0.2;

    // Collector Preset Positions
    final int    NOT_MOVING = 5;
    final int    LIFTER_PRESSED   =   15 ;
    final int    LIFTER_PICKUP    =   35 ;
    final int    LIFTER_HOVER     =  110 ;
    final int    LIFTER_STANDBY   =  930 ;
    final int    LIFTER_UP_TOP    = 1000 ;
    final int    LIFTER_DUMP_DROP = 1070 ;
    final int    LIFTER_THROW     =  120 ;

    final int    GRABBER_OPEN     =  130 ;
    final int    GRABBER_WIDE     =  170 ;

    // Declare OpMode members.
    private ElapsedTime stateTime = new ElapsedTime();
    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;
    private DcMotor grabber     = null;
    private DcMotor lifter      = null;
    private DcMotorSimple dumper = null;
    private DigitalChannel dumperReady = null;

    private boolean lastGrabberButton = false;
    private boolean lastLifterButton  = false;
    private boolean lastSafeButton    = false;
    private int currentLifterSetpoint = 0;

    private CollectorState currentCollectorState = CollectorState.UNKNOWN;

    // VUFORIA variables
    public VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables targetsUltimateGoal;
    private static final String VUFORIA_KEY =
            "ASFl1ib/////AAABmdtl1FqwZUIEqtOW/F+xX70YsCPMRYbusW+Av5TpUTDuB3VJT4z6ju8tkAzSKLD0cIwdp/o/3ggJzx27+OsIHWn8OTNfsAtxIzQVSCa75gI76/v006khzWpGV1wmdoEgK7JkvEns6BCzmgfSBSThg70Ej42wDF7l5FuIXUhm/AAMJ7sHLlMl5BboZg/vRyNRFTbEbFLyj98DOwLlaNl9DvUtf5bGBOHwFCNOBX8vlxWVU3aZZpGNxNTX/KyZ84TWECIxg8SeRSz3QcBEwsBYX97HXfj4nJxn93u8m5SZmoHF11MPkV0tlqemRwrCy/MJ3eGB3WCJ+MEeCAYeVa30E+WEkVTiFQAo4WW3vKuEVuBc";

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Camera");
        telemetry.update();

        // Initialize the hardware variables
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.useExtendedTracking = false;

        telemetry.addData("Status", "Initializing Hardware");
        telemetry.update();

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

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

        dumper      = hardwareMap.get(DcMotorSimple.class, "dumper");
        dumper.setDirection(DcMotor.Direction.FORWARD);

        dumperReady = hardwareMap.get(DigitalChannel.class, "dumper_ready");

        // Home the collector
        if (!LifterStatus.lifterHomed) {
            homeCollector();
        } else {
            // Lock in current position, and set motor modes.
            setLifterTarget(lifter.getCurrentPosition());
            lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter.setPower(0.7);

            grabber.setTargetPosition(grabber.getTargetPosition());
            grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grabber.setPower(0.5);

            setCollectorState(CollectorState.STANDBY);
        }


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        stateTime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Manual Homing.
            if (gamepad1.back && gamepad1.start) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                homeCollector();
            }

            // ##  Driving Code  ##############################################################
            // POV Mode uses left stick to go forward, and right stick to turn.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            // Run standard or TURBO
            if (gamepad1.left_stick_button) {
                drive *= TURBO_DRIVE_RATE;
            } else {
                drive *= DRIVE_RATE;
            }

            if (gamepad1.right_stick_button) {
                turn *= TURBO_TURN_RATE;
            } else {
                turn *= TURN_RATE;
            }

            if (gamepad1.dpad_up) {
                drive = DRIVE_CRAWL;
            } else if (gamepad1.dpad_down) {
                drive = -DRIVE_CRAWL;
            } else if (gamepad1.dpad_right) {
                turn = TURN_CRAWL;
            } else if (gamepad1.dpad_left) {
                turn = -TURN_CRAWL;
            }

            double leftPower    = drive + turn;
            double rightPower   = drive - turn;
            
            // Normaloze power to motors.
            double max = Math.abs(Math.max(leftPower, rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // ##  Dumper Code  ##############################################################
            runDumperControl();

            // ##  Lifter  Code  ##############################################################
            runCollectorControl();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Collector", currentCollectorState.toString());
            telemetry.addData("State Time", "%3.1f", stateTime.time());
            telemetry.addData("Dumper", "%s", dumperReady.getState()? "Dumping": "Ready");
            telemetry.addData("Grabber", "%d", grabber.getCurrentPosition());
            telemetry.addData("Lifter", "%d", lifter.getCurrentPosition());
            telemetry.update();
        }

        LifterStatus.lifterHomed = false;
    }

    public void runCollectorControl() {
        switch (currentCollectorState) {
            case STANDBY:
                if (lifterClicked()) {
                    setLifterTarget(LIFTER_PICKUP);
                    grabber.setTargetPosition(GRABBER_OPEN);
                    setCollectorState(CollectorState.LOWERING);
                }
                break;

            case LOWERING:
                if (!(lifter.isBusy() || grabber.isBusy())) {
                    setCollectorState(CollectorState.READY);
                }
                break;

            case READY:
                if (grabberClicked()) {
                    // lifter.setTargetPosition(LIFTER_PRESSED);
                    grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    grabber.setPower(-0.30);
                    setCollectorState(CollectorState.MANUAL_GRABBING);
                } else if (lifterClicked()) {
                    if (currentLifterSetpoint <= LIFTER_PICKUP) {
                        setLifterTarget(LIFTER_PRESSED);
                        grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        grabber.setPower(-0.30);
                        setCollectorState(CollectorState.AUTO_GRABBING);
                    } else {
                        setLifterTarget(LIFTER_PRESSED);
                        setCollectorState(CollectorState.AUTO_GRABBING_PAUSE);
                    }
                } else if (gamepad1.right_trigger > 0.25) {
                    grabber.setTargetPosition(GRABBER_WIDE);
                } else {
                    checkSafeClick();
                }
                break;

            case MANUAL_GRABBING:
                if (lifterClicked()) {
                    grabber.setPower(-0.6);
                    setLifterTarget(LIFTER_DUMP_DROP);
                    setCollectorState(CollectorState.LIFTING);
                } else if (grabberClicked()) {
                    //lifter.setTargetPosition(LIFTER_PICKUP);
                    grabber.setPower(0.0);
                    grabber.setTargetPosition(GRABBER_OPEN);
                    grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabber.setPower(0.5);
                    setCollectorState(CollectorState.READY);
                } else {
                    checkSafeClick();
                }
                break;

            case AUTO_GRABBING_PAUSE:
                if (stateTime.time() > 0.25) {
                    grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    grabber.setPower(-0.30);
                    setCollectorState(CollectorState.AUTO_GRABBING);
                }
                break;

            case AUTO_GRABBING:
                // Wait for gripper to close then lift.
                if (stateTime.time() > 0.3) {
                    grabber.setPower(-0.6);
                    setLifterTarget(LIFTER_DUMP_DROP);
                    setCollectorState(CollectorState.LIFTING);
                }
                break;

            case LIFTING:
                if (lifter.getCurrentPosition() > (LIFTER_DUMP_DROP - LIFTER_THROW)) {
                    grabber.setPower(1.0);
                    grabber.setTargetPosition(GRABBER_OPEN);
                    grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabber.setPower(0.5);
                    setCollectorState(CollectorState.UP_TOP);
                } else if (lifterClicked()) {
                    grabber.setPower(0.5);
                    grabber.setTargetPosition(GRABBER_OPEN);
                    grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setLifterTarget(LIFTER_PICKUP);
                    setCollectorState(CollectorState.LOWERING);
                }
                break;

            case UP_TOP:
                if (lifterClicked()) {
                    setLifterTarget(LIFTER_PICKUP);
                    setCollectorState(CollectorState.LOWERING);
                } else if (safeClicked()) {
                    setLifterTarget(LIFTER_HOVER);
                    setCollectorState(CollectorState.LOWERING);
                } else if (stateTime.time() > 0.25) {
                    setLifterTarget(LIFTER_UP_TOP);
                }
                break;

            case DUMPING:
                if (stateTime.time() > 0.25) {
                    setLifterTarget(LIFTER_PICKUP);
                    setCollectorState(CollectorState.LOWERING);
                }
                break;

            case UNKNOWN:
            default:
                break;
        }
    }

    void setLifterTarget(int newPos) {
        lifter.setTargetPosition(newPos);
        currentLifterSetpoint = newPos;
    }

    void checkSafeClick() {
        if (safeClicked()) {
            if (currentLifterSetpoint == LIFTER_PICKUP) {
                setLifterTarget(LIFTER_HOVER);
            } else {
                setLifterTarget(LIFTER_PICKUP);
            }
        }
    }

    public void runDumperControl() {
        //  Raise the dumper if left bumper pressed.
        if (gamepad1.x) {
            dumper.setPower(-0.4);
        } else {
            // Run the motor backwards unless it's in it's home position
            if (dumperReady.getState()) {
                dumper.setPower(0.2);
            } else {
                dumper.setPower(0);
            }
        }
    }
    public void setCollectorState(CollectorState newState) {
        currentCollectorState = newState;
        stateTime.reset();
    }

    public boolean grabberClicked() {
        boolean now = gamepad1.right_bumper;
        boolean clicked = (now && !lastGrabberButton) ;
        lastGrabberButton = now;
        return clicked;
    }

    public boolean lifterClicked() {
        boolean now = gamepad1.left_bumper;
        boolean clicked = (now && !lastLifterButton) ;
        lastLifterButton = now;
        return clicked;
    }

    public boolean safeClicked() {
        boolean now = gamepad1.a;
        boolean clicked = (now && !lastSafeButton) ;
        lastSafeButton = now;
        return clicked;
    }

    public void homeCollector() {
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
            runDumperControl();
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
            runDumperControl();
        }

        // Move to standby position.

        lifter.setPower(0.0);
        setLifterTarget(LIFTER_STANDBY);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.7);

        grabber.setPower(0.0);
        grabber.setTargetPosition(GRABBER_OPEN);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber.setPower(0.5);

        setCollectorState(CollectorState.STANDBY);
        LifterStatus.lifterHomed = true;

        telemetry.addData("Collector", "STANDBY");
        telemetry.update();
    }
}
