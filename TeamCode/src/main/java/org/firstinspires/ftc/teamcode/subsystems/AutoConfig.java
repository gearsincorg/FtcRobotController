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

  public static int MENU_ITEMS = 8;

  public class Param {
      public boolean redAlliance    = false;
      public boolean startFront     = false;
      public int delayStart         = 0;
      public boolean doubleYellow   = false;
      public int delayYellow        = 0;
      public boolean parkCenter     = false;
      public boolean whitePixel     = false;
      public boolean spare2         = false;

      //public List<AutoMenuItem> menuItems = new ArrayList<>(LOCATION_ITEMS);
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
        outputStreamWriter.write(Boolean.toString(autoOptions.startFront)  + "\n");
        outputStreamWriter.write(Integer.toString(autoOptions.delayStart)   + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.doubleYellow)  + "\n");
        outputStreamWriter.write(Integer.toString(autoOptions.delayYellow)   + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.parkCenter)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.whitePixel)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.spare2)  + "\n");

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
        autoOptions.startFront = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.delayStart = Integer.valueOf(bufferedReader.readLine());
        autoOptions.doubleYellow = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.delayYellow = Integer.valueOf(bufferedReader.readLine());
        autoOptions.parkCenter = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.whitePixel = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.spare2 = Boolean.valueOf(bufferedReader.readLine());
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
      myOpMode.telemetry.addData((currentMenuIndex == 1) ? "1 > START POSITION"   : "1   Start", autoOptions.startFront ? "At Front" : "At Back");
      myOpMode.telemetry.addData((currentMenuIndex == 2) ? "2 > START DELAY"   : "2   Start Delay", autoOptions.delayStart);
      myOpMode.telemetry.addData((currentMenuIndex == 3) ? "3 > YELLOW PIXEL"   : "3   Yellow Pixel", autoOptions.doubleYellow ? "Double" : "Single");
      myOpMode.telemetry.addData((currentMenuIndex == 4) ? "4 > YELLOW DELAY"   : "4   Yellow Delay", autoOptions.delayYellow);
      myOpMode.telemetry.addData((currentMenuIndex == 5) ? "5 > PARK"   : "5  Park", autoOptions.parkCenter ? "In Center" : "By Wall");
      myOpMode.telemetry.addData((currentMenuIndex == 6) ? "6 > WHITE PIXEL"   : "6  White Pixel", autoOptions.whitePixel ? "YES" : "no");
      myOpMode.telemetry.addData((currentMenuIndex == 7) ? "7 > SPARE 1"   : "7  Spare 2", autoOptions.spare2 ? "YES" : "no");
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
                    autoOptions.startFront = !autoOptions.startFront;
                    break;
                case 2:
                    if (b1)
                        autoOptions.delayStart++;
                    else
                    if (autoOptions.delayStart > 0)
                        autoOptions.delayStart--;
                    break;
                case 3:
                    autoOptions.doubleYellow = !autoOptions.doubleYellow;
                    break;
                case 4:
                    if (b1)
                        autoOptions.delayYellow++;
                    else
                    if (autoOptions.delayYellow > 0)
                        autoOptions.delayYellow--;
                    break;
                case 5:
                    autoOptions.parkCenter = !autoOptions.parkCenter;
                    break;
                case 6:
                    autoOptions.whitePixel = !autoOptions.whitePixel;
                    break;
                case 7:
                    autoOptions.spare2 = !autoOptions.spare2;
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
