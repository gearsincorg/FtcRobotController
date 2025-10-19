package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeSubsystem {

    private boolean showTelemetry = false;
    private boolean enabled = false;
    private LinearOpMode myOpMode;
    private DcMotor intake;

    private final double INTAKE_POWER = 0.5;

    public IntakeSubsystem(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize the Subsystem by creating hardware devices.
     * @param showTelemetry
     */
    public void init(boolean showTelemetry) {
        this.showTelemetry = showTelemetry;
        this.enabled = true;

        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(){
        // skip if not initialized
        if (!enabled) return;

        if (myOpMode.gamepad1.dpadUpWasPressed()){
            intake.setPower(INTAKE_POWER);
        } else {
            intake.setPower(0.0);
        }
    }
}
