package org.firstinspires.ftc.teamcode.subsystems;

public class ColorTarget {

    public boolean valid;
    public double x;
    public double y;

    public ColorTarget() {
        valid = false;
        x = 0;
        y = 0;
    }

    public ColorTarget(double x, double y) {
        valid = true;
        this.x = x;
        this.y = y;
    }
}
