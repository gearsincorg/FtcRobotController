package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {

    private static final double DRONE_FIRED = 1;
    private static final double DRONE_READY = 0;

    private DcMotor launcher;
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

        fire = myOpMode.hardwareMap.get(Servo.class, "fire");
        fire.setPosition(DRONE_READY);

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public void runLauncher( ){
        launcher.setPower(0.6);
    }

    public void stopLauncher( ){
        fire.setPosition(DRONE_READY);
        launcher.setPower(0);
    }

    public void fireDrone (){
        fire.setPosition(DRONE_FIRED);
    }
}
