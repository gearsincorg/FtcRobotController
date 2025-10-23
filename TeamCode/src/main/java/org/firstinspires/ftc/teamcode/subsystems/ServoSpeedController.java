package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class ServoSpeedController {

    // class members
    LinearOpMode  myOpMode;
    String        myName;
    int           OQPosIdx;
    int           OQVelIdx;

    public ServoSpeedController(LinearOpMode myOpMode, String servoName, int OQPositionIndex, int OQVelocityIndex) {
        this.myOpMode = myOpMode;
        this.myName   = servoName;
        this.OQPosIdx = OQPositionIndex;
        this.OQVelIdx = OQVelocityIndex;
    }

    // Constants
    static final double QUADCOUNTS_PER_REV = 8196.0;
    static final double ABSCOUNTS_PER_REV  = 1024.0;
    static final double PWMCOUNTS2DEGREES = 360.0 / ABSCOUNTS_PER_REV;
    static final double QUADCOUNTS2DEGREES = 360.0 / QUADCOUNTS_PER_REV;

    // process members
    int         quad_rev_offset;
    boolean     enabled = false;
    CRServo     servo;
    double      currentVel = 0;
    double      currentAng = 0;
    double      targetVel  = 0;
    double      targetAng  = 0;

    public void init(boolean showTelemetry) {
        enabled = true;
        servo = myOpMode.hardwareMap.get(CRServo.class, myName);
        servo.setPower(0);

        // determine the QUAD offset to offset
    }

    public void update() {

    }

    public void setVelTarget(double velDPS) {

    }

    public void setAngTarget(double angDEG, double maxVelDPS) {

    }

    public void stop() {

    }

    public double getAng() {
        currentAng = (double)Globals.OQ_POSITIONS[OQPosIdx] * COUNTS2DEGREES;
        return currentAng;
    }

    public double getVelDPS() {
        return currentVel;
    }
}
