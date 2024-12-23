/* Copyright (c) 2019 G-FORCE.
 *
 * This Class is used for the Path Planning Menu system
 * It manages the on-screen menu system.
 *
 */

package org.firstinspires.ftc.teamcode.subsystems;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class AutoConfig
{
    Context context;
    OpMode myOpMode;

    // these names MUST match the order found in Autonomous OpMode
    public String[] autoArray = new String[] {"SPEC 4-Sub",                // 0
                                              "SPEC 5-Sub",                // 1
                                              "SAMP 1-Basket Sub",         // 2
                                              "SAMP 1-Basket 2-Net Sub" ,  // 3
                                              "SPEC 1-Sub SAMP 3-Basket"}; // 4
    int autoModes = autoArray.length;
    public static int MENU_ITEMS = 3;

    public class Param {
      public boolean redAlliance    = false;
      public int delayStart         = 0;
      public int autoMode           = 0;
    }

    public int currentMenuIndex;
    public Param autoOptions;

    // variables used during the configuration process
    //AutoMenuItem currentMenuItem;
    boolean prev;
    boolean x1;
    boolean b1;
    boolean next;
    boolean lastPrev;
    boolean lastX1;
    boolean lastB1;
    boolean lastNext;
    private String configFileName="GFORCE.txt";

    public AutoConfig(OpMode opMode)
    {
      myOpMode = opMode;
      autoOptions = new Param();
    }

    public void saveConfig() {
        try {
              OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput(configFileName, Context.MODE_PRIVATE));

              // write each configuration parameter as a string on its own line
                outputStreamWriter.write(Boolean.toString(autoOptions.redAlliance)   + "\n");
                outputStreamWriter.write(Integer.toString(autoOptions.delayStart)   + "\n");
                outputStreamWriter.write(Integer.toString(autoOptions.autoMode)   + "\n");

              outputStreamWriter.close();
        }
        catch (IOException e) {
            myOpMode.telemetry.addData("Exception", "Auto Settings file write failed: " + e.toString());
        }
    }

    public void readConfig() {
    // read configuration data from file
    try
    {
      InputStream inputStream = context.openFileInput(configFileName);

      if (inputStream != null)
      {
        InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
        BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

        autoOptions.redAlliance = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.delayStart = Integer.valueOf(bufferedReader.readLine());
        autoOptions.autoMode = Integer.valueOf(bufferedReader.readLine());
        inputStream.close();
      }
    } catch (Exception e)
        {
          myOpMode.telemetry.addData("Config", "Blank Config.");
        }
    }

    public void updateMenu ()
    {
        myOpMode.telemetry.addData((currentMenuIndex == 0) ? "0 > ALLIANCE"   : "0   Alliance", autoOptions.redAlliance ? "RED" : "BLUE");
        myOpMode.telemetry.addData((currentMenuIndex == 1) ? "1 > START DELAY"   : "1   Start Delay", autoOptions.delayStart);
        myOpMode.telemetry.addData((currentMenuIndex == 2) ? "2 > AUTO MODE"    : "2   Auto Mode", autoArray[autoOptions.autoMode]);
    }

    public void initialize() {
        context  = myOpMode.hardwareMap.appContext;

        // Get the current auto configuration
        currentMenuIndex = 0;
        readConfig();

        // setup initial toggle memory states for buttons used
        lastPrev =false;
        lastX1   =false;
        lastB1   =false;
        lastNext =false;
    }

    public void runMenuUI() {

        // read the gamepad state
        prev = myOpMode.gamepad1.dpad_up;
        x1 = myOpMode.gamepad1.dpad_left;
        b1 = myOpMode.gamepad1.dpad_right;
        next = myOpMode.gamepad1.dpad_down;

        // checking to see if we are switching to the next menu item.
        if (next && !lastNext) {
            // move to next menu item
            currentMenuIndex = (currentMenuIndex + 1 ) % MENU_ITEMS;
        }
        // checking to see if we are switching to the prev menu item.
        else if (prev && !lastPrev) {
            // move to prev menu item
            currentMenuIndex = (currentMenuIndex + MENU_ITEMS - 1 ) % MENU_ITEMS;
        }
        // checking if we are moving to the next menu item.
        else if ((b1 && !lastB1) || (x1 && !lastX1)) {
            // select next option
            switch (currentMenuIndex) {
                case 0:
                    autoOptions.redAlliance = !autoOptions.redAlliance;
                    break;
                case 1:
                    if (b1) {
                        autoOptions.delayStart++;
                    } else if (autoOptions.delayStart > 0) {
                        autoOptions.delayStart--;
                    }
                    break;
                case 2:
                    if (b1) {
                        if (autoOptions.autoMode < autoModes - 1) {
                            autoOptions.autoMode++;
                        }
                    } else if (autoOptions.autoMode > 0) {
                        autoOptions.autoMode--;
                    }
                    break;
            }
            saveConfig();
        }
        updateMenu();

        // update toggle memory for next call
        lastPrev = prev;
        lastX1 = x1;
        lastB1 = b1;
        lastNext  = next;
    }
}
