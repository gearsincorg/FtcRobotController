package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.AutoConfig;

public class Robot {
    public IntakeSubsystem intake;
    public VisionSubsystem vision;
    public ShooterSubsystem shooter;
    public Follower drive;
    public AutoConfig config;

    private OpMode  opMode;

    public Robot(OpMode opmode) {
        this.opMode = opmode;
    }

    public void initialize() {
        vision = new VisionSubsystem(opMode);
        vision.init();

        // Setup robot
        drive = Constants.create(opMode.hardwareMap);

        // Shooter aims from odometry, so it needs the drive's Follower.
        shooter = new ShooterSubsystem(opMode, drive);
        shooter.init();

        // Intake waits for the shooter to be ready before feeding each ball.
        intake = new IntakeSubsystem(opMode, shooter::isReady);
        intake.init();

        // Start on the alliance chosen in the auto setup menu (Share / Options still override).
        config = new AutoConfig(opMode);
        config.initialize();
    }

    /***
     * Update any required robot functions.
     */
    public void update() {
        drive.update();
    }
}
