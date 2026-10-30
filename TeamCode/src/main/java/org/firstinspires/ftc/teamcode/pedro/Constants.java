package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.config.Configuration;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro constants for both drive chassis.
 *
 * The chassis is chosen from the name of the active robot configuration (res/xml/prototype.xml
 * or res/xml/production.xml), so the same build runs on either robot.  Anything that depends on
 * drive geometry or gearing (motor and pod directions, pod offsets, heading scalar, Foresight
 * tuning) has one copy per chassis; motor names and OctoQuad ports are shared.
 */
public class Constants {

    public enum Chassis { PROTOTYPE, PRODUCTION }

    public static Follower create(HardwareMap h) {
        return create(h, ForesightConfig.Constraint.NONE);
    }

    /** Same, with path following capped at maxPathSpeed (inches/sec).  Only this follower is affected. */
    public static Follower create(HardwareMap h, double maxPathSpeed) {
        ForesightConfig foresight = foresightConfig();
        foresight.maxPathSpeed.set(maxPathSpeed);
        return new Follower(
            new OctoQuadLocalizer(h, localizerConfig()),
            new Mecanum(h, drivetrainConfig()),
            new Foresight(foresight)
        );
    }

    /** Which chassis we're on, from the active robot configuration name. */
    public static Chassis chassis() {
        String name = new RobotConfigFileManager().getActiveConfig().getName();
        if (name.equalsIgnoreCase("prototype"))  return Chassis.PROTOTYPE;
        if (name.equalsIgnoreCase("production")) return Chassis.PRODUCTION;
        throw new IllegalStateException("Active robot configuration is \"" + name
                + "\". Select \"prototype\" or \"production\" so the right drive constants are used.");
    }

    public static MecanumConfig drivetrainConfig() {
        return chassis() == Chassis.PRODUCTION ? PRODUCTION_DRIVETRAIN : PROTOTYPE_DRIVETRAIN;
    }

    public static OctoQuadConfig localizerConfig() {
        return chassis() == Chassis.PRODUCTION ? PRODUCTION_LOCALIZER : PROTOTYPE_LOCALIZER;
    }

    /** A new config each call, so a speed cap set on one follower doesn't leak into the next opmode. */
    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(chassis() == Chassis.PRODUCTION ? PRODUCTION_FORESIGHT : PROTOTYPE_FORESIGHT);
    }

    // == Shared ================================================================

    /** Drive motor settings that are the same on both chassis (directions differ by gearing). */
    private static void sharedDrivetrain(MecanumConfig c) {
        c.frontLeftName.set("front_left_drive");
        c.frontRightName.set("front_right_drive");
        c.backLeftName.set("back_left_drive");
        c.backRightName.set("back_right_drive");
        c.manualBrakeMode.set(true);
    }

    /** OctoQuad settings that are the same on both chassis. */
    private static void sharedLocalizer(OctoQuadConfig c) {
        c.name.set("octoquad");
        c.xPodPort.set(0);
        c.yPodPort.set(1);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.ticksPerUnit.set(505.316944406);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
    }

    // == Prototype =============================================================

    // Integral term for the last inch, added to the prototype's secondary (under 2.5 in)
    // translational controllers.  Accumulates only within END_I_ZONE inches of the target,
    // shrinks by END_I_DECAY each loop, and is capped so it adds at most END_I_GAIN * END_I_MAX
    // power (0.15).  If the robot overshoots or wobbles at the end, lower END_I_GAIN.
    private static final double END_I_GAIN  = 0.3;
    private static final double END_I_ZONE  = 1.0;    // inches
    private static final double END_I_DECAY = 0.98;   // per loop
    private static final double END_I_MAX   = 0.5;    // inch-seconds

    /** A new integral controller each call: each one keeps its own running total. */
    private static Controller endIntegral() {
        return Controller.integral(() -> END_I_GAIN, () -> END_I_ZONE, () -> END_I_DECAY, () -> END_I_MAX);
    }

    // Prototype gearing: left side reversed.
    private static final MecanumConfig PROTOTYPE_DRIVETRAIN = new MecanumConfig(c -> {
        sharedDrivetrain(c);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    private static final OctoQuadConfig PROTOTYPE_LOCALIZER = new OctoQuadConfig(c -> {
        sharedLocalizer(c);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.xPodOffset.set(-1.5748031496062993);
        c.yPodOffset.set(0.2362204724409449);
        c.headingScalar.set(1.0053120122844978);
    });

    private static final Configuration<ForesightConfig> PROTOTYPE_FORESIGHT = (
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.18257094263290366);
            Controller secondaryTranslationalForward = Controller.proportional(0.06745508491742425)
                    .plus(endIntegral());
            Controller primaryTranslationalLateral = Controller.proportional(0.2716341796782243);
            Controller secondaryTranslationalLateral = Controller.proportional(0.10036157119214668)
                    .plus(endIntegral());

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.016096688576042517));
            c.brake.set(Controller.proportionalFeedforward(0.01368218528963614));

            c.headingFeedback.set(Controller.proportional(2.5926570763148415));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03214582249785932, 0.009670479088120737));

            c.linearBrakeCoefficients.set(Matrix.diag(0.0361039729948627, 0.025073372963681693));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0022049326116299955, 0.002346840747657526));

            c.maxAchievableForwardVelocity.set(62.95392399917404);
            c.maxAchievableStrafeVelocity.set(34.43322584962535);
            c.naturalForwardDeceleration.set(33.75201289593079);
            c.naturalStrafeDeceleration.set(47.4271060723835);

            // End of path: hold with full translational gain (default 0.45 was too weak to move the
            // last couple of inches), and give it up to 500 ms (default 100) to settle before the
            // path command finishes and the next one starts.
            c.holdPointTranslationalScaling.set(1.0);
            c.timeoutConstraint.set(500.0);
        }
    );

    // == Production ============================================================
    // TODO tune: copied from the prototype.  Re-run the OctoQuad and Foresight tuners on the
    // production chassis and paste the results here.

    // Production gearing: right side reversed.
    private static final MecanumConfig PRODUCTION_DRIVETRAIN = new MecanumConfig(c -> {
        sharedDrivetrain(c);
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    private static final OctoQuadConfig PRODUCTION_LOCALIZER = new OctoQuadConfig(c -> {
        sharedLocalizer(c);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.xPodOffset.set(4.429133858267717);
        c.yPodOffset.set(-0.610236220472441);
        c.headingScalar.set(1.009115985239217);
    });

    private static final Configuration<ForesightConfig> PRODUCTION_FORESIGHT = (
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.17514564795165746);
            Controller secondaryTranslationalForward = Controller.proportional(0.06471163694023177);
            // TODO re-tune: the tuner gave -33019.67 / -12199.88 (huge and negative: the robot strafed
            // off as soon as a path started).  These are the prototype's values until re-tuned.
            Controller primaryTranslationalLateral = Controller.proportional(0.2716341796782243);
            Controller secondaryTranslationalLateral = Controller.proportional(0.10036157119214668);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.011007764268495888));
            c.brake.set(Controller.proportionalFeedforward(0.009356599628221504));

            //c.headingFeedback.set(Controller.proportional(2.4287153624364843));
            c.headingFeedback.set(Controller.proportional(2.0));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.02956147155921096, 0.00748505210139825));

            c.linearBrakeCoefficients.set(Matrix.diag(0.06371873554530225, 0.05871431963111268));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011784821162332333, 0.0013118615445462875));
          //  c.maxAchievableForwardVelocity.set(93.689720623814);
        //    c.maxAchievableStrafeVelocity.set(64.51240737912634);
            c.maxAchievableForwardVelocity.set(20.0);
            c.maxAchievableStrafeVelocity.set(20.0);
            c.naturalForwardDeceleration.set(34.77902604546239);
            c.naturalStrafeDeceleration.set(57.88843024205049);
        }
    );
}
