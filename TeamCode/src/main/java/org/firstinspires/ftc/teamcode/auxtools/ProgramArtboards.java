package org.firstinspires.ftc.teamcode.auxtools;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.subsystems.PrismSubsystem;

@TeleOp(name="Dustin Artboard Loader", group="Linear OpMode")
@Disabled

public class ProgramArtboards extends LinearOpMode{
    GoBildaPrismDriver prism;

    PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();

   @Override public void runOpMode(){
       prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
       prism.setStripLength(30);

       // load the three animations into each artboard.
   }
}
