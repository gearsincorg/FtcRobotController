package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

public class FlowerIntake {
    private final DcMotorEx flower1;
    private final DcMotorEx flower2;
    private OpMode opMode;

    public FlowerIntake(OpMode myOpmode) {
        flower1 =myOpmode.hardwareMap.get(DcMotorEx.class, "flower1");
        flower2 = myOpmode.hardwareMap.get(DcMotorEx.class, "flower2");
        flower1.setDirection(DcMotorSimple.Direction.REVERSE);
        opMode = myOpmode;
    }

    public void intake_stop(){
        flower1.setPower(0);
        flower2.setPower(0);
        opMode.telemetry.addData("intake","stop");
        opMode.telemetry.update();
    }

    public void intake_start(){
        flower1.setPower(1);
        flower2.setPower(1);
        opMode.telemetry.addData("intake","start");
        opMode.telemetry.update();
    }

    public void intake_reverse(){
        flower1.setPower(-1);
        flower2.setPower(-1);
        opMode.telemetry.addData("intake","reverse");
        opMode.telemetry.update();
    }

    // Put commands here  ==========================================

    public Command offCommand() {
        return new InstantCommand(() -> intake_stop());
    }

    public Command onCommand() {
        return new InstantCommand(() -> intake_start());
    }

    public Command reverseCommand() {
        return new InstantCommand(() ->intake_reverse());
    }

}



