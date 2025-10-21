/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name="GFORCE Autonomous", group = "AA" ,  preselectTeleOp="GFORCE Teleop")
public class GFORCEAutonomous extends LinearOpMode
{
    DriveSubsystem driveSubsystem = new DriveSubsystem( this);
    AutoConfig autoConfig   = new AutoConfig(this);

    private Action selectedAuto  = null;
    private int lastSelectedAuto = -1;

    private final double START_X = 62;
    private final double START_Y = 16;
    private final double START_H = Math.toRadians(180);


    // Place all auto builders here!
    //================================================================================================================

    private Action build_TestDrive() {
        driveSubsystem.setPose(new Pose2d(START_X, START_Y, START_H));

        Action testDrive = driveSubsystem.actionBuilder(new Pose2d(START_X, START_Y, START_H))
                .splineTo(new Vector2d(0, 54), Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-24, 24), Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(START_X, START_Y), Math.toRadians(0))
                .build();

        return new SequentialAction(
                // Score Specimen 1 then sweep 3 more, score 4
                testDrive
        );
    }

    // ############################################################################

    @Override
    public void runOpMode()
    {
        Globals.IS_AUTO = true;
        autoConfig.initialize();
        driveSubsystem.init(new Pose2d(0,0,0), true);
        selectedAuto = loadSelectedAuto(autoConfig.autoOptions.autoMode);  // build the current auto sequence

        // Wait for driver to press start
        while(opModeInInit()) {

            autoConfig.runMenuUI(); // Run menu system
            if (autoConfig.autoOptions.autoMode != lastSelectedAuto) {
                lastSelectedAuto = autoConfig.autoOptions.autoMode;
                selectedAuto = loadSelectedAuto(autoConfig.autoOptions.autoMode);
            }

            // Set GLOBAL flags based on menu choices.
            if (autoConfig.autoOptions.redAlliance )
                Globals.ALLIANCE_COLOR = AllianceColor.RED;
            else
                Globals.ALLIANCE_COLOR = AllianceColor.BLUE;

            telemetry.addLine("\n Touch Play to run Auto");
            telemetry.update();
        }

        Globals.OCTO_ERRORS = 0;

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Do a count down if these is a delayed start,
            for (int sec = autoConfig.autoOptions.delayStart; sec > 0; sec--) {
                telemetry.addData("AUTO MODE",  "%s", autoConfig.autoArray[autoConfig.autoOptions.autoMode]);
                telemetry.addData("COUNTDOWN",  "%d  %d  %d  %d", sec, sec, sec, sec);
                telemetry.update();
                sleep(1000);
            }

            if (selectedAuto != null) {
                Actions.runBlocking(selectedAuto);
            } else {
                telemetry.addData("AUTO MODE",  "No valid mode selected");
                telemetry.update();
            }
        }

        Globals.LAST_POSE = driveSubsystem.getPose() ;
    }

    /**
     * Take the current auto mode and build the matching RadRunner sequence.
     * Save the last auto value for outside comparison.
     * @param autoMode
     * @return
     */
    private Action loadSelectedAuto(int autoMode) {

        Action sequentialAction = new SequentialAction();
        lastSelectedAuto = autoMode;

        // Note:  The cases below MUST match the order of auto options in the AutoConfig.java file.
        switch (autoMode) {
            case 0:
                sequentialAction = build_TestDrive();
                break;
        }

        // Run 4 actions simultaniously
        return  new ParallelAction(
                //arm.actionUpdate(),
                //lift.actionUpdate(),
                sequentialAction
                //arm.actionUpdateTelemetry()
        );
    }
}
