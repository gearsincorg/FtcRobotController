/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Autonomous opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="!Competition")
public class Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    public AutoConfig   autoConfig    = new AutoConfig();
    public RobotHardware robot         = new RobotHardware();

    public static final String TAG = "G-FORCE";

    private ElapsedTime autoTime = new ElapsedTime();

    int skyStonePosition = 0;
    boolean isRed;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        autoConfig.init(hardwareMap.appContext, this);
        robot.init(this);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested()) {
            autoConfig.init_loop(); //Run menu system
            sleep(20);
        }

        // Get alliance color from Menu
        isRed = autoConfig.autoOptions.redAlliance;

        if (autoConfig.autoOptions.redAlliance) {
            robot.allianceColor = RobotHardware.AllianceColor.RED;
        } else {
            robot.allianceColor = RobotHardware.AllianceColor.BLUE;
        }

        //Starting autonomous reset heading to zero
        robot.readSensors();
        autoTime.reset();

        if (autoConfig.autoOptions.enabled) {
        }
        robot.stopRobot();
    }

}