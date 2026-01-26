package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.auxtools.Datalogger;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;

import java.time.LocalTime;
import java.time.format.DateTimeFormatter;

// ... inside your activity or method

public class LoggingSubsystem extends SubsystemBase {

    static LoggingSubsystem.Datalog datalog = null;
    VoltageSensor battery;

    public LoggingSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    // General Subsystem Members

    @Override
    public void init(boolean showTelemetry) {
        super.init(showTelemetry);  // do not remove

        // Get devices from the hardwareMap.
        battery = myOpMode.hardwareMap.voltageSensor.get("Control Hub");

        // Get the current time without a specific time zone
        LocalTime currentTime = LocalTime.now();

        // Format the time into a readable string
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("MM_dd-HH_mm_ss");
        String formattedTime = currentTime.format(formatter);

        // Initialize the datalog
        datalog = new LoggingSubsystem.Datalog("datalog_" + formattedTime + ".txt");
    }

    @Override
    public void readSensors() {
        datalog.opModeMode.set(Globals.IS_AUTO ? "AUTO" : "TELE");
        datalog.opModeStatus.set(myOpMode.opModeInInit()? "INIT" : "RUN");
        datalog.robot.set(Globals.ROBOT_STATE.toString());
        datalog.spindexer.set(Globals.SPINDEXER_STATE.toString());
        datalog.turret.set(Globals.TURRET_STATE.toString());

        datalog.battery.set(battery.getVoltage());
    }

    @Override
    public  void runProcessing() {
        // The logged timestamp is taken when writeLine() is called.
        datalog.writeLine();
    }

    @Override
    public void runStateMachine() {

    }

    public static void updateCycle(double cycle) {
        if (datalog != null) {
            datalog.cycle.set("%.1f", cycle);
        }
    }

    public static void updateWheelSpeed(double front, double back) {
        if (datalog != null) {
            datalog.frontWheel.set(front);
            datalog.backWheel.set(back);
        }
    }

    public static void updateColorSelection(int patternID, int currentAutoShot, ArtifactColor seeking,
                                            int green, int purple) {
        if (datalog != null) {
            datalog.patternID.set(patternID);
            datalog.currentAutoShot.set(currentAutoShot);
            datalog.seeking.set(seeking.toString());
            datalog.green.set(green);
            datalog.purple.set(purple);
        }
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
        public Datalogger.GenericField cycle        = new Datalogger.GenericField("Cycle");
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("Status");
        public Datalogger.GenericField opModeMode   = new Datalogger.GenericField("Mode");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");
        public Datalogger.GenericField robot        = new Datalogger.GenericField("Robot");
        public Datalogger.GenericField spindexer    = new Datalogger.GenericField("Spindexer");
        public Datalogger.GenericField turret       = new Datalogger.GenericField("Turret");
        public Datalogger.GenericField frontWheel   = new Datalogger.GenericField("Front Wheel");
        public Datalogger.GenericField backWheel    = new Datalogger.GenericField("Back Wheel");

        public Datalogger.GenericField patternID    = new Datalogger.GenericField("patternID");
        public Datalogger.GenericField currentAutoShot = new Datalogger.GenericField("currentAutoShot");
        public Datalogger.GenericField seeking      = new Datalogger.GenericField("seeking");
        public Datalogger.GenericField green        = new Datalogger.GenericField("green");
        public Datalogger.GenericField purple       = new Datalogger.GenericField("purple");

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
                    cycle,
                    opModeMode,
                    opModeStatus,
                    robot,
                    spindexer,
                    patternID,
                    currentAutoShot,
                    seeking,
                    green,
                    purple,
                    turret,
                    frontWheel,
                    backWheel,
                    battery
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
