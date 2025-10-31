package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auxtools.SharedOQ;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    public TurretSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // subsystem devices
    private VisionSubsystem visionSubsystem;
    private DcMotor aim;
    private DcMotor shoot;

    // Subsystem Constants
    private final double DEADBAND = 1.0;
    private final double OUTPUT_LIMIT = 0.75;
    private final double GAIN = 0.005;
    private final double WARNING = 140;
    private final int ONE_ROTATION = 537;
    private final double COUNTS_PER_DEGREES = 537.5 / 360;
    private final double RED_X = -1482;
    private final double RED_Y = -1413;
    private final double BLUE_X = -1482;
    private final double BLUE_Y = -1413;
    private final double SPIN_LIMIT = 180;

    // Subsystem Speed/Power constants


    // Servo positions

    // General Subsystem Members
    private double error = 0;
    private boolean resetting = false;

    private double Aa = 0;
    private double Ar = 0;
    private double Ae = 0;
    private double At = 0;
    private double Ad = 0;

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);  // do not remove

        aim = myOpMode.hardwareMap.get(DcMotor.class, "aim");
        aim.setDirection(DcMotorSimple.Direction.FORWARD);
        aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoot = myOpMode.hardwareMap.get(DcMotor.class, "shooter");
        shoot.setDirection(DcMotorSimple.Direction.FORWARD);

        // initialize the vision subsystem
        visionSubsystem.init(true);
    }

    @Override
    public void readSensors(){
        //turret angle, robot heading, calculate the AprilTag Angle
        calculateAd();
    }

    @Override
    public void runStateMachine(){

        if (myOpMode.gamepad1.rightBumperWasPressed()) {
            int targetPosition;

            if (Math.abs(At) > SPIN_LIMIT) {
                resetting = true;
                if (At > 0){
                    targetPosition = aim.getCurrentPosition() - ONE_ROTATION;
                } else {
                    targetPosition = aim.getCurrentPosition() + ONE_ROTATION;
                }

                aim.setTargetPosition(targetPosition);
                aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                aim.setPower(1.0);
            }
        }

        if (resetting){
            if (!aim.isBusy()){
                aim.setPower(0.0);
                aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                resetting = false;
            }
        } else {
            visionSubsystem.update();
            double bearing = visionSubsystem.getBearing();
            double output = 0;

            error = -bearing;
            if (Math.abs(error) > DEADBAND){
                output = (error * GAIN) - (myOpMode.gamepad1.right_stick_x * 0.15);
                output = Range.clip(output, -OUTPUT_LIMIT, OUTPUT_LIMIT);
            }

            //converts encoder clicks to degrees
            At = (aim.getCurrentPosition() / COUNTS_PER_DEGREES) % 360;

            aim.setPower(output);

            if (Math.abs(At) > WARNING){
                myOpMode.gamepad1.rumble(500);
            }
        }
    }

    @Override
    public void showStatus(){
        myOpMode.telemetry.addData("turret degrees", At);
    }

    private void calculateAd(){
        double x = RED_X - SharedOQ.OQlocalizer.posX_mm;
        double y = RED_Y - SharedOQ.OQlocalizer.posY_mm;
        Aa = Math.atan2(y, x);
        Ar = Math.toDegrees(SharedOQ.OQlocalizer.heading_rad);
        Ad = Aa - Ar;
    }
}
