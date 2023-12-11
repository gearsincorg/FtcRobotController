/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.Datalogger;

@Disabled
@TeleOp(name = "Lancher Speed Test", group = "Datalogging")
public class LauncherSpeedTest extends LinearOpMode
{
    Datalog datalog;
    VoltageSensor battery;
    DcMotorEx launcher = null;
    ElapsedTime runtime = new ElapsedTime();
    
    double voltage1      = 0;
    double voltage2      = 0;
    double[]  cps = new double[10];    

    final double INITIAL_POWER = 0.5;
    final double FINAL_POWER   = 0.90;
    final double POWER_STEP    = 0.05;
    
    
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get devices from the hardwareMap.
        battery = hardwareMap.voltageSensor.get("Control Hub");
        launcher  = hardwareMap.get(DcMotorEx.class, "lateral");
        launcher.setDirection(DcMotorEx.Direction.FORWARD);

        // Initialize the datalog
        datalog = new Datalog("speedtest");
        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        runtime.reset();
        datalog.start.set(INITIAL_POWER);
        datalog.step.set(POWER_STEP);

        //Sample the initial Voltage
        voltage1= battery.getVoltage();

        while (opModeIsActive() && (voltage1 > 11.0))
        {
            //Sample the initial Voltage
            voltage1= battery.getVoltage();

            // Cycle moto through power levels
            int powerIndex = 0;
            for (double power = INITIAL_POWER; (power <= FINAL_POWER) && (powerIndex < 10); power += POWER_STEP, powerIndex++ ) {
                launcher.setPower(power);
                sleep(2000);
                cps[powerIndex] = launcher.getVelocity() ;            

                // Datalog fields are stored as text only; do not format here.
                telemetry.addData("Voltage", battery.getVoltage());
                telemetry.addData("CPS", cps[powerIndex]);
                telemetry.addData("RPM", cps[powerIndex] * 60 / 28);
                telemetry.update();
            }
            voltage2= battery.getVoltage();
            
            // Record datapoint.
            datalog.cps0.set(cps[0]);
            datalog.cps1.set(cps[1]);
            datalog.cps2.set(cps[2]);
            datalog.cps3.set(cps[3]);
            datalog.cps4.set(cps[4]);
            datalog.cps5.set(cps[5]);
            datalog.cps6.set(cps[6]);
            datalog.cps7.set(cps[7]);
            datalog.cps8.set(cps[8]);
            datalog.cps9.set(cps[9]);
            datalog.voltage1.set(voltage1);
            datalog.voltage2.set(voltage2);

            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();

            // turn off motor
            launcher.setPower(0);
            sleep(50000);
        }

        /*
         * The datalog is automatically closed and flushed to disk after 
         * the OpMode ends - no need to do that manually :')
         */
    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;
        
        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField start = new Datalogger.GenericField("start");
        public Datalogger.GenericField step  = new Datalogger.GenericField("step");
        public Datalogger.GenericField voltage1 = new Datalogger.GenericField("voltage1");
        public Datalogger.GenericField voltage2 = new Datalogger.GenericField("voltage2");
        public Datalogger.GenericField cps0    = new Datalogger.GenericField("cps 0.50");
        public Datalogger.GenericField cps1    = new Datalogger.GenericField("cps 0.55");
        public Datalogger.GenericField cps2    = new Datalogger.GenericField("cps 0.60");
        public Datalogger.GenericField cps3    = new Datalogger.GenericField("cps 0.65");
        public Datalogger.GenericField cps4    = new Datalogger.GenericField("cps 0.70");
        public Datalogger.GenericField cps5    = new Datalogger.GenericField("cps 0.75");
        public Datalogger.GenericField cps6    = new Datalogger.GenericField("cps 0.80");
        public Datalogger.GenericField cps7    = new Datalogger.GenericField("cps 0.85");
        public Datalogger.GenericField cps8    = new Datalogger.GenericField("cps 0.90");
        public Datalogger.GenericField cps9    = new Datalogger.GenericField("cps 0.95");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

            // Pass through the filename
            .setFilename(name)

            // Request an automatic timestamp field
            .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

            // Tell it about the fields we care to log.
            // Note that order *IS* important here! The order in which we list
            // the fields is the order in which they will appear in the log.
            .setFields(
                voltage1,
                voltage2,
                cps0,
                cps1,
                cps2,
                cps3,
                cps4,
                cps5,
                cps6,
                cps7,
                cps8,
                cps9,
                start,
                step
            )
            .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}
