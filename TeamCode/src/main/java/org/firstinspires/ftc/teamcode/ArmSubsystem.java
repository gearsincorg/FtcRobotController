package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArmSubsystem {
    private DcMotor arm;      //motor used to control the arm
    private LinearOpMode myOpMode;
    private boolean showTelemetry     = false;

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
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Requires motor encoder cables to be hooked up.

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    /**
     * set the power of the arm
     * positive is up
     * @param power
     */
    public void setPower(double power){
        arm.setPower(-power);
    }

    /**
     * stop the arm from moving
     */
    public void stop(){
        arm.setPower(0);
    }



}
