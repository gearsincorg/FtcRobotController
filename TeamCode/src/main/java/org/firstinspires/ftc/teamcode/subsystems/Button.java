package org.firstinspires.ftc.teamcode.subsystems;

public class Button {
    private boolean lastState = false;

    public boolean pressed (boolean state) {
        boolean test = lastState;
        lastState = state;
        return (state && !test);
    }

    public boolean released (boolean state) {
        boolean test = lastState;
        lastState = state;
        return (!state && test);
    }
}
