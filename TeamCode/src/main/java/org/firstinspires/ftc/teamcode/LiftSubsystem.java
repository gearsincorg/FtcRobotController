package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LiftSubsystem {

    public final double MAX_HEIGHT = 41;
    public final double MIN_HEIGHT = 10;
    public final double SPECIMIN_HEIGHT = 10; // was 9, correct height
    public final double HIGH_CHAMBER = 27;
    public final double HIGH_CHAMBER_RELEASE = 19;
    public final double MANUAL_UP_POWER = 1;
    public final double MANUAL_DOWN_POWER = -0.3;
    public final double AUTO_UP_POWER = 1;
    public final double AUTO_DOWN_POWER = -0.8;
    private final double HOLD_POWER = 0.1;
    private final double HOME_POWER = -0.6;

    private final double SLOPE = 0.0123;
    private final double OFFSET = 9.5;
    private final int MINIMUM_MOVEMENT = 10;

    private DcMotor lift;      //motor used to control the lift
    private LinearOpMode myOpMode;
    private boolean showTelemetry     = false;
    private double setpointInches = 0;
    private double currentPosition = 0;
    private int lastPosition = 0;
    private boolean goingHome = false;

    // Arm Constructor
    public LiftSubsystem(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry){
        lift = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Requires motor encoder cables to be hooked up.

        homeTheLift();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void readSensors(){
        int encoderValue = lift.getCurrentPosition();
        currentPosition = (SLOPE * encoderValue) + OFFSET;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Lift Position", "%.1f inches", currentPosition);
        }
    }

    /**
     * set the power of the arm
     * positive is up
     * @param power
     */
    public void setPower(double power){
        lift.setPower(power);
    }

    /**
     * stop the arm from moving
     */
    public void stop(){
        lift.setPower(0);
    }

    public void hold(){
        lift.setPower(HOLD_POWER);
    }

    public void runLiftControl() {
        readSensors();
        double error = setpointInches - currentPosition;
        double power = 0;

        if(goingHome){

            int position = lift.getCurrentPosition();
            if(Math.abs(position-lastPosition) < MINIMUM_MOVEMENT){
                power = 0;
                resetEncoders();
                goingHome = false;
            } else {
                power = HOME_POWER;
            }
            lastPosition = position;
            myOpMode.sleep(100);
            setSetpointInches(currentPosition);

        } else {
            if ((error > 0.5) && (getCurrentPosition() < MAX_HEIGHT)) {
                power = AUTO_UP_POWER;
            } else if ((error < -0.5) && (getCurrentPosition() > MIN_HEIGHT)) {
                power = AUTO_DOWN_POWER;
            } else {
                power = HOLD_POWER;
            }
        }


        setPower(power);
        myOpMode.telemetry.addData("lift error",error);
        myOpMode.telemetry.addData("lift power", power);
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
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(10);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void homeTheLift(){
        goingHome = true;
        myOpMode.telemetry.addLine("homing the lift");
        myOpMode.telemetry.update();
        lift.setPower(HOME_POWER);
        myOpMode.sleep(250);

    }

}
