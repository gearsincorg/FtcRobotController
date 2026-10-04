package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlowerIntake {
    private final DcMotorEx flower1;
    private final DcMotorEx flower2;

    public FlowerIntake(HardwareMap hardwareMap) {
        flower1 = hardwareMap.get(DcMotorEx.class, "flower1");
        flower2 = hardwareMap.get(DcMotorEx.class, "flower2");
        flower1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void intake_stop(){
        flower1.setPower(0);
        flower2.setPower(0);
    }

    private void intake_start(){
        flower1.setPower(1);
        flower2.setPower(1);
    }

    private void intake_reverse(){
        flower1.setPower(-1);
        flower2.setPower(-1);
    }

    public Command off() {
        return instant(() -> intake_stop());
    }

    public Command on() {
        return instant(() -> intake_start());
    }

    public Command reverse() {

        return instant(() ->intake_reverse());
    }
}



