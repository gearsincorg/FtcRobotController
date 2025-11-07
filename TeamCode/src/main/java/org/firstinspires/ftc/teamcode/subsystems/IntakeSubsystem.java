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
    private double power = 0;

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);

        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    /**
     *  Run any non-state machine pre-processing
     *  Called every update() Cycle
     */
    public void runProcessing() {
        if (myOpMode.gamepad1.dpad_up){
            power = INTAKE_POWER;
        } else {
            power = 0.0;
        }
        intake.setPower(power);
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("Intake", "%s Power %.1f", currentState, power);
    }


}
