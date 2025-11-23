package org.firstinspires.ftc.teamcode.auxtools;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SubsystemBase {
    public LinearOpMode     myOpMode;
    public StateBase        currentState;
    public boolean          showTelemetry = false;
    public boolean          subsystemEnabled = false;
    ElapsedTime             stateTime = new ElapsedTime();

    public SubsystemBase (LinearOpMode myOpMode) {
        this.myOpMode = myOpMode;
    }

    public void init (boolean showTelemetry) {
        this.showTelemetry = showTelemetry;
        this.subsystemEnabled = true;
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

    public void runProcessing() {}

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
