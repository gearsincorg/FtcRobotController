package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    public IntakeSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private DcMotor intake;

    // Subsystem Speed/Power constants
    private final double INTAKE_POWER = 0.5;

    // Servo positions

    // General Subsystem Members

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);

        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runStateMachine(){
        if (myOpMode.gamepad1.dpad_up){
            intake.setPower(INTAKE_POWER);
        } else {
            intake.setPower(0.0);
        }
    }
}
