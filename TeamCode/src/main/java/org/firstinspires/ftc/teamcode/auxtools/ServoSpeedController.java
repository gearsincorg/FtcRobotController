package org.firstinspires.ftc.teamcode.auxtools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class ServoSpeedController {

    // class members
    LinearOpMode  myOpMode;
    String        myName;
    int           OQPosIdx;
    int           OQVelIdx;

    public ServoSpeedController(LinearOpMode myOpMode, String servoName, int OQPWM_Index, int OQQuadIndex) {
        this.myOpMode = myOpMode;
        this.myName   = servoName;
        this.OQPosIdx = OQPWM_Index;
        this.OQVelIdx = OQQuadIndex;
    }

    // Constants
    static final double QUADCOUNTS_PER_REV = 8192.0;
    static final double ABSCOUNTS_PER_REV  = 1024.0;
    static final double PWMCOUNTS2DEGREES = 360.0 / ABSCOUNTS_PER_REV;
    static final double QUADCOUNTS2DEGREES = 360.0 / QUADCOUNTS_PER_REV;
    static final double VEL_P = 0.002;
    static final double VEL_F = 543.6;
    static final double CPSP_2_DPS = 20 * QUADCOUNTS2DEGREES;

    // process members
    int         quad_rev_offset;
    boolean     enabled = false;
    CRServo     servo;
    double      currentVel = 0;
    double      currentAng = 0;
    double      targetVel  = 0;
    double      targetAng  = 0;
    double      outputPower = 0;

    public void init(boolean showTelemetry) {
        enabled = true;
        servo = myOpMode.hardwareMap.get(CRServo.class, myName);
        servo.setPower(0);

        // determine the QUAD offset to offset
        SharedOQ.init(myOpMode);
        SharedOQ.update();
    }

    public void update() {
        if (!DriveSubsystem.isEnabled()){
            SharedOQ.update();
        }

        //Calculate servo power needed to reach target velocity
        currentVel = SharedOQ.OQencoder.velocities[OQVelIdx] * CPSP_2_DPS;
        double error = targetVel - currentVel;
        outputPower = (VEL_P * error) + DPSToPower(targetVel);

        servo.setPower(outputPower);
        
        showStatus();

        myOpMode.telemetry.addData("control" ,"VELP %f  VELF %f", (VEL_P * error), (targetVel / VEL_F));
    }

    public void setVelTarget(double velDPS) {
        targetVel = velDPS;
    }

    public void setAngTarget(double angDEG, double maxVelDPS) {
        targetAng = angDEG;

    }

    public void stop() {
        setVelTarget(0);
    }

    public double getAng() {

        return currentAng;
    }

    public double getVelDPS() {
        return currentVel;
    }
    
    public void showStatus() {
        myOpMode.telemetry.addData("Servo Velocity", "Target: %f  Current: %f  Output: %f", targetVel, currentVel, outputPower);
    }

    public double DPSToPower(double DPS){
        return (DPS * 2.21E-03) +  (-9.33E-09 * Math.pow(DPS, 3));
    }
}
