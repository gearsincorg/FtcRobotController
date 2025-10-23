package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    public TurretSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private VisionSubsystem visionSubsystem;
    private DcMotor aim;
    private DcMotor shoot;

    // Subsystem Constants
    private final double DEADBAND = 1.0;
    private final double OUTPUT_LIMIT = 0.75;
    private final double GAIN = 0.005;
    private final double WARNING = 1.25;
    private final double COUNTS_PER_REVOLUTION = 537.5;

    // Subsystem Speed/Power constants

    // Servo positions

    // General Subsystem Members
    private double error = 0;
    private double turns = 0;
    private boolean resetting = false;

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);  // do not remove

        aim = myOpMode.hardwareMap.get(DcMotor.class, "aim");
        aim.setDirection(DcMotorSimple.Direction.FORWARD);
        aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoot = myOpMode.hardwareMap.get(DcMotor.class, "shooter");
        shoot.setDirection(DcMotorSimple.Direction.FORWARD);

        // initialize the vision subsystem
        visionSubsystem.init(true);
    }

    @Override
    public void runStateMachine(){

        if (myOpMode.gamepad1.rightBumperWasPressed()) {
            int targetPosition;

            if (Math.abs(turns) > 1.0) {
                resetting = true;
                if (turns > 0){
                    targetPosition = aim.getCurrentPosition() - (int)COUNTS_PER_REVOLUTION;
                } else {
                    targetPosition = aim.getCurrentPosition() + (int)COUNTS_PER_REVOLUTION;
                }

                aim.setTargetPosition(targetPosition);
                aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                aim.setPower(1.0);
            }
        }

        if (resetting){
            if (!aim.isBusy()){
                aim.setPower(0.0);
                aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                resetting = false;
            }
        } else {
            visionSubsystem.update();
            double bearing = visionSubsystem.getBearing();
            double output = 0;

            error = -bearing;
            if (Math.abs(error) > DEADBAND){
                output = (error * GAIN) - (myOpMode.gamepad1.right_stick_x * 0.15);
                output = Range.clip(output, -OUTPUT_LIMIT, OUTPUT_LIMIT);
            }

            //converts encoder clicks to revolutions
            turns = aim.getCurrentPosition() / COUNTS_PER_REVOLUTION;

            aim.setPower(output);

            if (Math.abs(turns) > WARNING){
                myOpMode.gamepad1.rumble(500);
            }
        }
    }

    @Override
    public void showStatus(){
        myOpMode.telemetry.addData("turret turns", turns);
    }
}
