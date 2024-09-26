package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArmSubsystem {

    private final double SLOPE = 0.0123;
    private final double OFFSET = 7.875;

    private DcMotor arm;      //motor used to control the arm
    private LinearOpMode myOpMode;
    private boolean showTelemetry     = false;
    private double setpointInches = 0;
    private double currentPosition = 0;


    // Arm Constructor
    public ArmSubsystem(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry){
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Requires motor encoder cables to be hooked up.

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void readSensors(){
        int encoderValue = arm.getCurrentPosition();
        currentPosition = (SLOPE * encoderValue) + OFFSET;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Arm Position", "%.1f inches", currentPosition);
        }
    }

    /**
     * set the power of the arm
     * positive is up
     * @param power
     */
    public void setPower(double power){
        arm.setPower(power);
    }

    /**
     * stop the arm from moving
     */
    public void stop(){
        arm.setPower(0);
    }

    public void hold(){
        arm.setPower(0.1);
    }

    public void runArmControl() {
        readSensors();
        double error = setpointInches - currentPosition;
    }
    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getSetpointInches() {
        return setpointInches;
    }

    public void setSetpointInches(double setpointInches) {
        this.setpointInches = setpointInches;
    }

    public void resetEncoders(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(10);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
