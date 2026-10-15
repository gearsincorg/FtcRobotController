package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class FlowerSubsystem extends SubsystemBase {
    private final OpMode opMode;
    private DcMotorEx flower1;
    private DcMotorEx flower2;
    private double power;

    // == standard Methods ===================================
    public FlowerSubsystem(OpMode myOpMode) {
        opMode = myOpMode;
    }

    public void init() {
        flower1 = opMode.hardwareMap.get(DcMotorEx.class, "flower1");
        flower2 = opMode.hardwareMap.get(DcMotorEx.class, "flower2");
        flower1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic(){
        showTelemetry();
    }

    void showTelemetry() {
        opMode.telemetry.addData("FLOWER","power = %4.0f", power);
    }

    // == new elements =========================================

    public void stop(){
        setPower(0);
    }

    public void start(){
        setPower(1);
    }

    public void reverse(){
        setPower(-1);
    }

    private void setPower(double newPower) {
        power = newPower;
        flower1.setPower(power);
        flower2.setPower(power);
    }

    // == Command methods  ==========================================

    public Command stopCommand() {
        return new InstantCommand(() -> stop());
    }

    public Command startCommand() {
        return new InstantCommand(() -> start());
    }

    public Command reverseCommand() {
        return new InstantCommand(() -> reverse());
    }

}



