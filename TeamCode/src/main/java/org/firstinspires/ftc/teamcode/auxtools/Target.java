package org.firstinspires.ftc.teamcode.auxtools;

public class Target {
    public boolean isValid ;
    public double  range ;
    public double  bearing ;

    public Target () {
        isValid = false;
        range = 0 ;
        bearing = 0;
    }

    public Target (double range, double bearing) {
        isValid = true;
        this.range = range ;
        this.bearing = bearing;
    }

    public Target (Target newTarget) {
        this.isValid = newTarget.isValid;
        this.range   = newTarget.range ;
        this.bearing = newTarget.bearing;
    }
}
