package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SubsystemBase {
    LinearOpMode    myOpMode;
    StateBase       currentState;
    boolean         showTelemetry = false;
    boolean         subsystemEnabled = false;
    ElapsedTime stateTimer = new ElapsedTime();

    public SubsystemBase (LinearOpMode myOpMode) {
        this.myOpMode = myOpMode;
    }

    public void init (boolean showTelemetry) {
        this.showTelemetry = showTelemetry;
        this.subsystemEnabled = true;
    }

    public void update(){
        readSensors();
        runStateMachine();
        if (showTelemetry) {
            showStatus();
        }
    }

    public void runStateMachine() {}

    public void setState(StateBase newState) {
        currentState = newState;
        stateTimer.reset();
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
