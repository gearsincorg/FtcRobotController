package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_drive");
        c.frontRightName.set("front_right_drive");
        c.backLeftName.set("back_left_drive");
        c.backRightName.set("back_right_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(1);
        c.yPodPort.set(0);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        // 2.x used 19.89 ticks/mm; 3.0 wants ticks per inch (19.89 * 25.4)
        c.ticksPerUnit.set(19.89 * 25.4);
        c.encoderResolutionUnit.set(DistanceUnit.INCH);
        // 2.x used 1 mm for both (tcpOffsetXMM was set twice, so Y was never set) - re-measure!
        c.xPodOffset.set(DistanceUnit.MM.toInches(1));
        c.yPodOffset.set(DistanceUnit.MM.toInches(1));
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.headingScalar.set(360.0 / 358.1);
    });

    // Only the values carried over from 2.x are set here (xVelocity/yVelocity).
    // Everything else (braking, translational/heading gains, etc.) comes from running
    // the Foresight Tuner, which prints a complete ForesightConfig to paste over this one.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.maxAchievableForwardVelocity.set(68.9);
        c.maxAchievableStrafeVelocity.set(62.79);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new OctoQuadLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
