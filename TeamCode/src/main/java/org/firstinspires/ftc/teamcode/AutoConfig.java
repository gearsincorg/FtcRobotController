/* Copyright (c) 2019 G-FORCE.
 *
 * This Class is used for the Path Planning Menu system
 * It manages the on-screen menu system.
 *
 */

package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class AutoConfig
{
  public Param autoOptions;

  public class Param {
      public boolean redAlliance = false;
      public boolean enabled = true;
      public boolean startFront = false;
      public boolean scoreJunction = false;
      public boolean scoreConeStack = false;
      public boolean park = false;
  }

  private static int MENU_ITEMS = 6;

  // variables used during the configuration process
  //AutoMenuItem currentMenuItem;
  private boolean prev;
  private boolean x1;
  private boolean b1;
  private boolean next;
  private boolean lastPrev;
  private boolean lastX1;
  private boolean lastB1;
  private boolean lastNext;
  private String configFileName="GFORCE.txt";
  private int currentMenuIndex;
  private LinearOpMode myOpMode;
  private Context context;

  public AutoConfig() {
    autoOptions = new Param();
  }

  public void init(LinearOpMode opMode) {

    this.myOpMode = opMode;
    this.context = opMode.hardwareMap.appContext;

    // Get the current auto configuration
    currentMenuIndex = 0;
    readConfig();

    // setup initial toggle memory states for buttons used
    lastPrev =false;
    lastX1   =false;
    lastB1   =false;
    lastNext =false;
  }

  public boolean init_loop() {

    // read the gamepad state
    prev = myOpMode.gamepad1.dpad_up;
    x1 = myOpMode.gamepad1.dpad_left;
    b1 = myOpMode.gamepad1.dpad_right;
    next = myOpMode.gamepad1.dpad_down;
    boolean change = false;

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
              autoOptions.enabled = !autoOptions.enabled;
              break;
          case 2:
              autoOptions.startFront = !autoOptions.startFront;
              break;
          case 3:
              autoOptions.scoreJunction = !autoOptions.scoreJunction;
              break;
          case 4:
              autoOptions.scoreConeStack = !autoOptions.scoreConeStack;
              break;
          case 5:
              autoOptions.park = !autoOptions.park;
              break;
      }
      saveConfig();
      change = true;
    }
    updateMenu();

    // update toggle memory for next call
    lastPrev = prev;
    lastX1 = x1;
    lastB1 = b1;
    lastNext  = next;

    return change;
  }

  private void saveConfig() {
    try {
      OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput(configFileName, Context.MODE_PRIVATE));

      // write each configuration parameter as a string on its own line
        outputStreamWriter.write(Boolean.toString(autoOptions.redAlliance)   + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.enabled)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.startFront)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.scoreJunction)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.scoreConeStack)  + "\n");
        outputStreamWriter.write(Boolean.toString(autoOptions.park)  + "\n");

      outputStreamWriter.close();
    }
    catch (IOException e) {
      myOpMode.telemetry.addData("Exception", "Auto Settings file write failed: " + e.toString());
    }
  }

  private void readConfig() {
    // read configuration data from file
    try
    {
      InputStream inputStream = context.openFileInput(configFileName);

      if (inputStream != null)
      {
        InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
        BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

        autoOptions.redAlliance = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.enabled = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.startFront = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.scoreJunction = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.scoreConeStack = Boolean.valueOf(bufferedReader.readLine());
        autoOptions.park = Boolean.valueOf(bufferedReader.readLine());
        inputStream.close();
      }
    } catch (Exception e)
    {
      myOpMode.telemetry.addData("Config", "Blank Config.");
    }
  }

  private void updateMenu ()
  {
      myOpMode.telemetry.addData((currentMenuIndex == 0) ? "0 > ALLIANCE"   : "0   Alliance", autoOptions.redAlliance ? "RED" : "blue");
      myOpMode.telemetry.addData((currentMenuIndex == 1) ? "1 > RUN AUTO"   : "1   Run Auto", autoOptions.enabled ? "YES" : "no");
      myOpMode.telemetry.addData((currentMenuIndex == 2) ? "2 > START POSITION"   : "2   Start Position", autoOptions.startFront ? "FRONT" : "rear");
      myOpMode.telemetry.addData((currentMenuIndex == 3) ? "3 > SCORE JUNCTION"   : "3   Score Junction", autoOptions.scoreJunction ? "YES" : "no");
      myOpMode.telemetry.addData((currentMenuIndex == 4) ? "4 > SCORE CONESTACK"   : "4  Score Conestack", autoOptions.scoreConeStack ? "YES" : "no");
      myOpMode.telemetry.addData((currentMenuIndex == 5) ? "5 > PARK"   : "5  Park", autoOptions.park ? "YES" : "no");
      myOpMode.telemetry.update();
  }
}
