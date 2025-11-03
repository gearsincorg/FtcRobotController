package org.firstinspires.ftc.teamcode.auxtools;

import static com.qualcomm.hardware.digitalchickenlabs.OctoQuad.I2cRecoveryMode.MODE_2_M1_PLUS_SCL_IDLE_ONESHOT_TGL;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SharedOQ {

    // OctoQuad constants
    private static final float TICKS_PER_MM = 19.89f;
    private static final float X_OFFSET_FROM_CENTER_MM =   80.0f;
    private static final float Y_OFFSET_FROM_CENTER_MM =  185.0f;
    private static final float OQ_IMU_SCALAR = (float)(360.0/348.66);
    private static final int OQ_PORT_X = 0;
    private static final int OQ_PORT_Y = 1;

    public static OctoQuad.LocalizerDataBlock OQlocalizer = new OctoQuad.LocalizerDataBlock();
    public static OctoQuad.EncoderDataBlock   OQencoder   = new OctoQuad.EncoderDataBlock();

    static OctoQuad oq = null;
    static LinearOpMode myOpMode = null;

    public static void init(LinearOpMode opMode) {
        if (myOpMode == null) {
            myOpMode = opMode;
            if (oq == null) {
                oq = myOpMode.hardwareMap.get(OctoQuad.class, "octoquad");
                intializeOctoQuad(oq);
            }
        }
    }

    public static void update() {
        if (oq != null) {
            oq.readLocalizerDataAndAllEncoderData(OQlocalizer, OQencoder);
        }
    }

    public static void resetEncoder(int channel){
        if (oq != null) {
            oq.resetSinglePosition(channel);
        }
    }

    public static void setLocalizerPose(Pose2d newPose) {
        if (oq != null) {
            oq.setLocalizerPose(inchToMm(newPose.position.x), inchToMm(newPose.position.y), (float) newPose.heading.toDouble());
        }
    }

    private static void intializeOctoQuad(OctoQuad oq) {
        oq.resetEverything();
        oq.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);

        // Configure the localizer
        oq.setSingleEncoderDirection(OQ_PORT_X, OctoQuad.EncoderDirection.FORWARD);
        oq.setSingleEncoderDirection(OQ_PORT_Y, OctoQuad.EncoderDirection.FORWARD);

        oq.setLocalizerPortX(OQ_PORT_X);
        oq.setLocalizerPortY(OQ_PORT_Y);
        oq.setLocalizerCountsPerMM_X(TICKS_PER_MM);
        oq.setLocalizerCountsPerMM_Y(TICKS_PER_MM);
        oq.setLocalizerTcpOffsetMM_X(X_OFFSET_FROM_CENTER_MM);
        oq.setLocalizerTcpOffsetMM_Y(Y_OFFSET_FROM_CENTER_MM);
        oq.setLocalizerImuHeadingScalar(OQ_IMU_SCALAR);
        oq.setLocalizerVelocityIntervalMS(50);

        oq.setI2cRecoveryMode(MODE_2_M1_PLUS_SCL_IDLE_ONESHOT_TGL);
        oq.resetLocalizerAndCalibrateIMU();
        oq.resetSinglePosition(0);
        oq.resetSinglePosition(1);
        oq.saveParametersToFlash();
    }

    private static int inchToMm(double inches) {
        return (int)(inches * 25.4);
    }

}
