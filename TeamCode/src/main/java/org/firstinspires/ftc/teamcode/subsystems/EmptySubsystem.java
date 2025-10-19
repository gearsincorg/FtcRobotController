package org.firstinspires.ftc.teamcode.subsystems;

import static android.provider.SyncStateContract.Helpers.update;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EmptySubsystem {

    private boolean showTelemetry;
    LinearOpMode myOpMode;
    boolean enabled = false;
    private ElapsedTime stateTime       = new ElapsedTime();
    // private EmptyStates  currentState    = INIT;

    /**
     * Initialize the Subsystem by creating hardware devices.
     * @param showTelemetry
     */
    public void init(boolean showTelemetry) {
        this.showTelemetry = showTelemetry;
        this.enabled = true;

    }

    public void update() {
        // skip if not initialized
        if (!enabled) return;
    }

    /*
    public void setState (EmptyStates newState){
        currentState = newState;
        stateTime.reset();
    }
    */


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
