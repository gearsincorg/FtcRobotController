package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystem {

    private boolean showTelemetry;
    private LinearOpMode myOpmode;
    //private VisionSubsystem visionSubsystem;

    private final double DEADBAND = 1.0;
    private final double OUTPUT_LIMIT = 0.75;
    private final double GAIN = 0.005;
    private final double WARNING = 1.25;
    private final double OVERIDE = 1.75;
    private final double COUNTS_PER_REVOLUTION = 537.5;


    private double error = 0;
    private double turns = 0;
    private boolean resetting = false;

    private DcMotor rotate;

    public TurretSubsystem(LinearOpMode opmode) {
        myOpmode = opmode;
        //visionSubsystem = new VisionSubsystem(myOpmode);
    }

    /**
     * Initialize the Subsystem by creating hardware devices.
     * @param showTelemetry
     */
    public void init(boolean showTelemetry) {
        this.showTelemetry = showTelemetry;

        rotate = myOpmode.hardwareMap.get(DcMotor.class, "rotate");
        rotate.setDirection(DcMotorSimple.Direction.FORWARD);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // initialize the vision subsystem
        // visionSubsystem.init(true);
    }

    public void update(){
        if (myOpmode.gamepad1.rightBumperWasPressed()) {
            int targetPosition;

            if (Math.abs(turns) > 1.0) {
                resetting = true;
                if (turns > 0){
                    targetPosition = rotate.getCurrentPosition() - (int)COUNTS_PER_REVOLUTION;
                } else {
                    targetPosition = rotate.getCurrentPosition() + (int)COUNTS_PER_REVOLUTION;
                }

                rotate.setTargetPosition(targetPosition);
                rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotate.setPower(1.0);
            }
        }

        if (resetting){
            if (!rotate.isBusy()){
                rotate.setPower(0.0);
                rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                resetting = false;
            }
        } else {
            //visionSubsystem.update();
            //double bearing = visionSubsystem.getBearing();
            double output = 0;

            //error = -bearing;
            if (Math.abs(error) > DEADBAND){
                output = (error * GAIN) - (myOpmode.gamepad1.right_stick_x * 0.15);
                output = Range.clip(output, -OUTPUT_LIMIT, OUTPUT_LIMIT);
            }

            //converts encoder clicks to revolutions
            turns = rotate.getCurrentPosition() / COUNTS_PER_REVOLUTION;

            rotate.setPower(output);
            if (showTelemetry){
                myOpmode.telemetry.addData("turret turns", turns);
            }

            if (Math.abs(turns) > WARNING){
                myOpmode.gamepad1.rumble(500);
            }
        }
    }

}
