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
        .rightFrontMotorName("front_right_drive")
        .rightRearMotorName("back_right_drive")
        .leftRearMotorName("back_left_drive")
        .leftFrontMotorName("front_left_drive")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .useBrakeModeInTeleOp(true)
        .xVelocity(68.9)
        .yVelocity(62.79)
        ;

    public static OctoQuadConstants localizerConstants = new OctoQuadConstants()
        .name("octoquad")
        .deadwheelPortX(1)
        .deadwheelPortY(0)
        .deadwheelXDir(OctoQuad.EncoderDirection.FORWARD)
        .deadwheelYDir(OctoQuad.EncoderDirection.REVERSE)
        .deadwheelXTicksPerMM(19.89f)
        .deadwheelYTicksPerMM(19.89f)
        .tcpOffsetXMM(1)
        .tcpOffsetXMM(1)
        .imuScalar((float)(360.0/358.1))
         ;


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .setLocalizer(new OctoQuadLocalizer(hardwareMap, localizerConstants, OctoQuadLocalizer.InitMode.INITIALIZE_OCTOQUAD))
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .build();
    }
}


