package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Values for TurretTuneOpMode that can be edited live in Panels (http://192.168.43.1:8001,
 * Configurables).  Panels edits the non-final static fields of a class marked @Configurable.
 * The gains are pushed to the shooter every loop, so an edit takes effect at once.  Starting
 * values are the shooter's defaults; when tuning is done, paste the final gains into the
 * ShooterSubsystem TURRET_* constants (nothing here is saved).
 */
@Configurable
public class TurretTuneConfig {
    // Live turret gains (radians and motor power; see ShooterSubsystem.TURRET_KP).
    public static double KP = ShooterSubsystem.TURRET_KP;   // power per rad of error
    public static double KI = ShooterSubsystem.TURRET_KI;   // power per rad of error per second
    public static double KD = ShooterSubsystem.TURRET_KD;   // power per rad/s of turret speed
    public static double KS = ShooterSubsystem.TURRET_KS;   // friction push, in the direction of the error
    public static double KV = ShooterSubsystem.TURRET_KV;   // power per rad/s of chassis turn

    // Tests
    public static double STEP_DEG        = 5;     // closed-loop step size (+-); raise once the tuner is trusted
    public static double WINDOW_DEG      = 15;    // every test stays within this of where it started
    public static double CHECK_POWER     = 0.2;   // power for the first "does it move" pulse

    // Gain design (from the identified plant)
    public static double BANDWIDTH_RAD_S = 20;    // wanted closed-loop natural frequency (capped by loop rate)
    public static double DAMPING         = 0.9;   // wanted damping ratio
}
