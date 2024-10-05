package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArmSubsystem {

    public final double MAX_HEIGHT = 41;
    public final double MIN_HEIGHT = 5.5;
    public final double SPECIMIN_HEIGHT = 10; // was 9, corret height
    public final double HIGH_CHAMBER = 27;
    public final double HIGH_CHAMBER_RELEASE = 19;
    public final double MANUAL_UP_POWER = 1;
    public final double MANUAL_DOWN_POWER = -0.3;
    public final double AUTO_UP_POWER = 1;
    public final double AUTO_DOWN_POWER = -0.8;
    private final double HOLD_POWER = 0.1;
    private final double HOME_POWER = -0.3;

    private final double SLOPE = 0.0123;
    private final double OFFSET = 5.0;
    private final int MINIMUM_MOVEMENT = 10;

    private DcMotor arm;      //motor used to control the arm
    private LinearOpMode myOpMode;
    private boolean showTelemetry     = false;
    private double setpointInches = 0;
    private double currentPosition = 0;
    private int lastPosition = 0;

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

        homeTheArm();

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
        arm.setPower(HOLD_POWER);
    }

    public void runArmControl() {
        readSensors();
        double error = setpointInches - currentPosition;
        double power;

        if ((error > 0.5) && (getCurrentPosition() < MAX_HEIGHT)) {
            power = AUTO_UP_POWER;
        } else if ((error < -0.5) && (getCurrentPosition() > MIN_HEIGHT)) {
            power = AUTO_DOWN_POWER;
        } else {
            power = HOLD_POWER;
        }
        setPower(power);
        myOpMode.telemetry.addData("arm error",error);
        myOpMode.telemetry.addData("arm power", power);
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

    public void homeTheArm(){
        myOpMode.telemetry.addLine("homing the arm");
        myOpMode.telemetry.update();
        arm.setPower(HOME_POWER);
        myOpMode.sleep(250);
        while(!myOpMode.isStopRequested()){
            int position = arm.getCurrentPosition();
            if(Math.abs(position-lastPosition) < MINIMUM_MOVEMENT){
                arm.setPower(0);
                resetEncoders();
                break;
            }
            lastPosition = position;
            myOpMode.sleep(100);
        }
        myOpMode.telemetry.addLine("homing completed ! :)");
        readSensors();
        setSetpointInches(currentPosition);

    }

}
