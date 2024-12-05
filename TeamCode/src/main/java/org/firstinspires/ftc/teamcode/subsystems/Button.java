package org.firstinspires.ftc.teamcode.subsystems;

public class Button {
    private boolean lastState = false;

    public boolean pressed (boolean state) {
        if (Globals.IS_AUTO)
            return(false);

        boolean test = lastState;
        lastState = state;
        return (state && !test);
    }

    public boolean released (boolean state) {
        if (Globals.IS_AUTO)
            return(false);

        boolean test = lastState;
        lastState = state;
        return (!state && test);
    }
}
