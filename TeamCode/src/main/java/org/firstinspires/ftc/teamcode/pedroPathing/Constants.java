package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OctoQuadConstants;
import com.pedropathing.ftc.localization.localizers.OctoQuadLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
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
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .xVelocity(68.9)
        .yVelocity(62.79)
        ;

    public static OctoQuadConstants localizerConstants = new OctoQuadConstants()
        .name("octoquad")
        .deadwheelPortX(0)
        .deadwheelPortY(1)
        .deadwheelXDir(OctoQuad.EncoderDirection.FORWARD)
        .deadwheelYDir(OctoQuad.EncoderDirection.FORWARD)
        .deadwheelXTicksPerMM(1)
        .deadwheelYTicksPerMM(1)
        .tcpOffsetXMM(1)
        .tcpOffsetXMM(1)
        .imuScalar(1.0f)
         ;


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .setLocalizer(new OctoQuadLocalizer(hardwareMap, localizerConstants, OctoQuadLocalizer.InitMode.INITIALIZE_OCTOQUAD))
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .build();
    }
}


