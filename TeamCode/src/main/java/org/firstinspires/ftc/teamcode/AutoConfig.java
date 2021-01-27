/* Copyright (c) 2019 G-FORCE.
 *
 * This Class is used for the Path Planning Menu system
 * It manages the on-screen menu system.
 *
 */

package org.firstinspires.ftc.teamcode;

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
  OpMode opMode;

  public static int MENU_ITEMS = 9;

  public class Param {
      public boolean redAlliance = false;
      public int delayInSec = 0;
      public boolean enabled = true;
      public boolean startCenter = false;
      public boolean scoreWobble = false;
      public boolean scoreRingsHigh = false;
      public boolean scorePowerShot = false;
      public boolean park = false;
      public boolean spare = false;

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

  public AutoConfig() {
    autoOptions = new Param();
  }

  public void init(Context context, OpMode opMode) {

    this.context = context;
    this.opMode = opMode;

    // Get the current auto configuration
    currentMenuIndex = 0;
    readConfig();

    // setup initial toggle memory states for buttons used
    lastPrev =false;
    lastX1 =false;
    lastB1 =false;
    lastNext =false;
  }

  public void init_loop() {

    // read the gamepad state
    prev = opMode.gamepad1.dpad_up;
    x1 = opMode.gamepad1.dpad_left;
    b1 = opMode.gamepad1.dpad_right;
    next = opMode.gamepad1.dpad_down;

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
              if (b1)
                  autoOptions.delayInSec++;
              else
              if (autoOptions.delayInSec > 0)
                  autoOptions.delayInSec--;
              break;
          case 2:
              autoOptions.enabled = !autoOptions.enabled;
              break;
          case 3:
              autoOptions.startCenter = !autoOptions.startCenter;
              break;
          case 4:
              autoOptions.scoreWobble = !autoOptions.scoreWobble;
              break;
          case 5:
              autoOptions.scoreRingsHigh = !autoOptions.scoreRingsHigh;
              break;
          case 6:
              autoOptions.scorePowerShot = !autoOptions.scorePowerShot;
              break;
          case 7:
              autoOptions.park = !autoOptions.park;
              break;
          case 8:
              autoOptions.spare = !autoOptions.spare;
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

  public void saveConfig() {
    try {
      OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput(configFileName, Context.MODE_PRIVATE));

      // write each configuration parameter as a string on its own line
        outputStreamWriter.write(Boolean.toString(autoOptions.redAlliance)   + "\n");
        outputStreamWriter.write(Integer.toString(autoOptions.delayInSec)   + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.enabled)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.startCenter)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.scoreWobble)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.scoreRingsHigh)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.scorePowerShot)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.park)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.spare)  + "\n");

      outputStreamWriter.close();
    }
    catch (IOException e) {
      opMode.telemetry.addData("Exception", "Auto Settings file write failed: " + e.toString());
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
        autoOptions.delayInSec  = Integer.valueOf(bufferedReader.readLine());
        autoOptions.enabled = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.startCenter = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.scoreWobble = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.scoreRingsHigh = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.scorePowerShot = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.park = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.spare = Boolean.valueOf(bufferedReader.readLine());
        inputStream.close();
      }
    } catch (Exception e)
    {
      opMode.telemetry.addData("Config", "Blank Config.");
    }
  }

  public void updateMenu ()
  {
      opMode.telemetry.addData((currentMenuIndex == 0) ? "0 > ALLIANCE"   : "0   Alliance", autoOptions.redAlliance ? "RED" : "Blue");
      opMode.telemetry.addData((currentMenuIndex == 1) ? "1 > START DELAY"   : "1   Start Delay", autoOptions.delayInSec);
      opMode.telemetry.addData((currentMenuIndex == 2) ? "2 > RUN AUTO"   : "2   Run Auto", autoOptions.enabled ? "YES" : "no");
      opMode.telemetry.addData((currentMenuIndex == 3) ? "3 > START POSITION"   : "3   Start Position", autoOptions.startCenter ? "By WALL" : "By center");
      opMode.telemetry.addData((currentMenuIndex == 4) ? "4 > SCORE WOBBLE"   : "4   Score Wobble", autoOptions.scoreWobble ? "YES" : "no");
      opMode.telemetry.addData((currentMenuIndex == 5) ? "5 > SCORE HIGH GOAL"   : "5   Score High Goal", autoOptions.scoreRingsHigh ? "YES" : "no");
      opMode.telemetry.addData((currentMenuIndex == 6) ? "6 > SCORE POWERSHOT"   : "6  Score Powershot", autoOptions.scorePowerShot ? "YES" : "no");
      opMode.telemetry.addData((currentMenuIndex == 7) ? "7 > PARK"   : "7  Park", autoOptions.park ? "YES" : "No");
      opMode.telemetry.addData((currentMenuIndex == 8) ? "8 > SPARE"   : "8  Spare", autoOptions.spare ? "YES" : "no");
      opMode.telemetry.update();
  }
}
