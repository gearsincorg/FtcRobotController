package org.firstinspires.ftc.teamcode.subsystems;

import static android.provider.SyncStateContract.Helpers.update;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class EmptySubsystem {

    private boolean showTelemetry;
    LinearOpMode myOpMode;
    boolean      enabled = false;

    public EmptySubsystem(LinearOpMode opmode) {
        myOpMode = opmode;
    }

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
