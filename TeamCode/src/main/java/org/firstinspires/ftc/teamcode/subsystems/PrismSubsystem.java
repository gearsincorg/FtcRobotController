package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.auxtools.StateBase;

public class PrismSubsystem {
    public LinearOpMode     myOpMode;
    public StateBase currentState;
    public boolean          showTelemetry = false;
    public boolean          subsystemEnabled = false;
    ElapsedTime             stateTime = new ElapsedTime();



    public PrismSubsystem(LinearOpMode myOpMode) {
        this.myOpMode = myOpMode;
    }

    private GoBildaPrismDriver prism;

    PrismAnimations.Solid solidBlue = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();

    public void init (boolean showTelemetry) {
        this.showTelemetry = showTelemetry;
        this.subsystemEnabled = true;
        prism = myOpMode.hardwareMap.get(GoBildaPrismDriver.class, "prism");
    }

    public void update(){
        if (subsystemEnabled) {
            readSensors();
            runProcessing();
            runStateMachine();
            if (showTelemetry) {
                showStatus();
            }
        }
    }

    public void runProcessing() {
        if (myOpMode.gamepad1.right_bumper){
            prism.clearAllAnimations();
            prism.insertAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, rainbowSnakes);
        } else if (myOpMode.gamepad1.left_bumper) {
            prism.clearAllAnimations();
            prism.insertAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidBlue);
        }
    }

    public void runStateMachine() {}

    public void setState(StateBase newState) {
        currentState = newState;
        stateTime.reset();
    }

    public boolean timeInState(double waitTime){
        return (stateTime.time() >= waitTime);
    }

    public boolean isEnabled() {
        return subsystemEnabled;
    }

    public void readSensors() { }
    public void showStatus() { }

    //-------------------------------------------------------------------------
    // ACTION  methods
    //-------------------------------------------------------------------------

    public Action actionUpdate(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                update();
                return true;
            }
        };
    }
}
