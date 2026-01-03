package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PrismSubsystem;

@TeleOp(name="Dustins light test", group="Linear OpMode")

public class DUSTINLightTest extends LinearOpMode{
    private PrismSubsystem prism = new PrismSubsystem(this);

   @Override public void runOpMode(){
       prism.update();
   }
}
