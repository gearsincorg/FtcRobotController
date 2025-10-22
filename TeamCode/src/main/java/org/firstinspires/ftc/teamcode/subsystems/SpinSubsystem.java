package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SpinSubsystem extends SubsystemBase{

    public SpinSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    @Override
    public void init (boolean showTelemetry) {
        setState(SpinStates.INIT);
        super.init(showTelemetry);
    }

    @Override
    public void runStateMachine() {
        switch ((SpinStates)currentState) {

        }
    }

    @Override
    public void showStatus() {
        myOpMode.telemetry.addData("Spin", currentState);
    }

}
