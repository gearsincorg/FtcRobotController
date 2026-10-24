package org.firstinspires.ftc.teamcode.util;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.Properties;

/**
 * On-screen setup menu for the autonomous, run during init on the Driver Station.
 * Gamepad 1 D-pad: up/down choose an item, left/right change it.  Every change is saved to a
 * file on the Control Hub so the settings survive a restart (and teleop can read the alliance).
 */
public class AutoConfig {

    public enum StartLocation {
        AUDIENCE_1("Audience 1"), AUDIENCE_2("Audience 2"), FAR_1("Far 1"), FAR_2("Far 2");
        public final String label;
        StartLocation(String label) { this.label = label; }
    }

    public enum Park {
        NONE("None"), BASE("Base");
        public final String label;
        Park(String label) { this.label = label; }
    }

    public static final int MAX_DELAY_SEC = 15;
    public static final int MAX_CYCLES    = 2;

    private static final String FILE_NAME = "GFORCE_auto.properties";
    private static final int ITEMS = 5;     // alliance, delay, start, cycles, park

    // Settings
    public boolean       red      = true;
    public int           delaySec = 0;
    public StartLocation start    = StartLocation.AUDIENCE_1;
    public int           cycles   = 2;
    public Park          park     = Park.BASE;

    private final OpMode opMode;
    private Context context;
    private int  item;
    private boolean lastUp, lastDown, lastLeft, lastRight;

    public AutoConfig(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Load the saved settings (defaults if there are none). */
    public void initialize() {
        context = opMode.hardwareMap.appContext;
        item = 0;
        load();
    }

    /** Call every init loop: handles the D-pad and shows the menu (caller does telemetry.update()). */
    public void runMenu() {
        boolean up = opMode.gamepad1.dpad_up, down = opMode.gamepad1.dpad_down;
        boolean left = opMode.gamepad1.dpad_left, right = opMode.gamepad1.dpad_right;

        if (down && !lastDown)    item = (item + 1) % ITEMS;
        else if (up && !lastUp)   item = (item + ITEMS - 1) % ITEMS;
        else if (right && !lastRight) { change(+1); save(); }
        else if (left && !lastLeft)   { change(-1); save(); }

        lastUp = up; lastDown = down; lastLeft = left; lastRight = right;
        show();
    }

    private void change(int dir) {
        switch (item) {
            case 0: red = !red; break;
            case 1: delaySec = clamp(delaySec + dir, 0, MAX_DELAY_SEC); break;
            case 2: start = StartLocation.values()[wrap(start.ordinal() + dir, StartLocation.values().length)]; break;
            case 3: cycles = clamp(cycles + dir, 0, MAX_CYCLES); break;
            case 4: park = Park.values()[wrap(park.ordinal() + dir, Park.values().length)]; break;
        }
    }

    private void show() {
        opMode.telemetry.addLine("AUTO SETUP  (D-pad: up/down = item, left/right = change)");
        line(0, "Alliance",    red ? "RED" : "BLUE");
        line(1, "Start delay", delaySec + " s");
        line(2, "Start",       start.label);
        line(3, "Cycles",      String.valueOf(cycles));
        line(4, "Park",        park.label);
    }

    private void line(int index, String name, String value) {
        opMode.telemetry.addData((item == index ? "> " : "   ") + name, value);
    }

    // == Saving ==============================================================
    public void save() {
        try (OutputStream out = context.openFileOutput(FILE_NAME, Context.MODE_PRIVATE)) {
            Properties p = new Properties();
            p.setProperty("red", Boolean.toString(red));
            p.setProperty("delaySec", Integer.toString(delaySec));
            p.setProperty("start", start.name());
            p.setProperty("cycles", Integer.toString(cycles));
            p.setProperty("park", park.name());
            p.store(out, "G-FORCE auto setup");
        } catch (Exception e) {
            opMode.telemetry.addData("Auto setup", "save failed: " + e);
        }
    }

    private void load() {
        try (InputStream in = context.openFileInput(FILE_NAME)) {
            Properties p = new Properties();
            p.load(in);
            red      = Boolean.parseBoolean(p.getProperty("red", "true"));
            delaySec = clamp(parseInt(p.getProperty("delaySec"), 0), 0, MAX_DELAY_SEC);
            cycles   = clamp(parseInt(p.getProperty("cycles"), 2), 0, MAX_CYCLES);
            start    = parseEnum(StartLocation.class, p.getProperty("start"), StartLocation.AUDIENCE_1);
            park     = parseEnum(Park.class, p.getProperty("park"), Park.BASE);
        } catch (Exception e) {
            // No file yet (or unreadable): keep the defaults.
        }
    }

    private static int parseInt(String s, int fallback) {
        try { return Integer.parseInt(s.trim()); } catch (Exception e) { return fallback; }
    }

    private static <E extends Enum<E>> E parseEnum(Class<E> type, String s, E fallback) {
        try { return Enum.valueOf(type, s.trim()); } catch (Exception e) { return fallback; }
    }

    private static int clamp(int v, int lo, int hi) { return Math.max(lo, Math.min(hi, v)); }
    private static int wrap(int v, int n)           { return (v % n + n) % n; }
}
