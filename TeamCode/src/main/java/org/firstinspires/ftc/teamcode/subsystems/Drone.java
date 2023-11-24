package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {

    private static final double DRONE_FIRED = 1;
    private static final double DRONE_LOADED = 0;
    private static final double DRONE_STOWWED = 0;

    private DcMotor launcher;
    private Servo tilt;
    private Servo fire;


    private LinearOpMode myOpMode;
    private boolean showTelemetry = false;

    // Drone Constructor
    public Drone(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void initialize(boolean showTelemetry)
    {
        launcher = myOpMode.hardwareMap.get(DcMotor.class, "lateral");  // Launcher
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tilt = myOpMode.hardwareMap.get(Servo.class, "tilt");
        tilt.setPosition(DRONE_STOWWED);
        fire = myOpMode.hardwareMap.get(Servo.class, "fire");
        fire.setPosition(DRONE_LOADED);

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void runLauncher( ){
        launcher.setPower(1);
    }

    public void stopLauncher( ){
        launcher.setPower(0);
    }

    public void setTiltAngle (double angle){
        tilt.setPosition(angle);
    }

    public void fireDrone (){
        fire.setPosition(DRONE_FIRED);
    }




}
