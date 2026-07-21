package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName("rightfront")
        .rightRearMotorName("rightback")
        .leftRearMotorName("leftback")
        .leftFrontMotorName("leftfront")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
        .rightFrontMotorName("rightfront")
        .rightRearMotorName("rightback")
        .leftRearMotorName("leftback")
        .leftFrontMotorName("leftfront")
        .leftFrontEncoderDirection(Encoder.REVERSE)
        .leftRearEncoderDirection(Encoder.REVERSE)
        .rightFrontEncoderDirection(Encoder.FORWARD)
        .rightRearEncoderDirection(Encoder.FORWARD)
        .robotWidth(15)
        .robotLength(12)
        .forwardTicksToInches(0.0075)
        .strafeTicksToInches(0.035)
        .turnTicksToInches(0.0112)
        ;


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .driveEncoderLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .build();
    }
}
