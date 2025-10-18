package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class EmptySubsystem {

    private boolean showTelemetry;
    LinearOpMode myOpmode;

    public EmptySubsystem(LinearOpMode opmode) {
        myOpmode = opmode;
    }

    /**
     * Initialize the Subsystem by creating hardware devices.
     * @param showTelemetry
     */
    public void init(boolean showTelemetry) {
        this.showTelemetry = showTelemetry;
    }
}
