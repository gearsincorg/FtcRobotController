package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.EncoderDataBlock;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OctoQuadIF {

    static final float TICKS_PER_MM = 12.66f;
    static final float X_OFFSET_FROM_CENTER_MM = -97.05f;
    static final float Y_OFFSET_FROM_CENTER_MM = -156.70f;
    static final float OQ_IMU_SCALAR = 1.0323f;
    static final int OQ_PORT_X = 1;
    static final int OQ_PORT_Y = 2;

    private OctoQuad            oq;
    private EncoderDataBlock    encoderDataBlock = new EncoderDataBlock();
    private LinearOpMode        myOpMode;
    private boolean             initialized = false;

    // Arm Constructor
    public OctoQuadIF(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void update() {
        if (!initialized) {
            // Connect to the OctoQuad by looking up its name in the hardwareMap.
            // Clear out all prior settings and encoder data before setting up desired configuration
            oq = myOpMode.hardwareMap.get(OctoQuad.class, "octoquad");
            oq.resetEverything();

            // Set the first 4 channels as relative encoders and the next 4 as absolute encoders
            oq.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);

            // Configure the localizer
            oq.setSingleEncoderDirection(OQ_PORT_X, OctoQuadBase.EncoderDirection.FORWARD);
            oq.setSingleEncoderDirection(OQ_PORT_Y, OctoQuadBase.EncoderDirection.REVERSE);
            oq.setLocalizerPortX(OQ_PORT_X);
            oq.setLocalizerPortY(OQ_PORT_Y);
            oq.setLocalizerCountsPerMM_X(TICKS_PER_MM);
            oq.setLocalizerCountsPerMM_Y(TICKS_PER_MM);
            oq.setLocalizerTcpOffsetMM_X(X_OFFSET_FROM_CENTER_MM);
            oq.setLocalizerTcpOffsetMM_Y(Y_OFFSET_FROM_CENTER_MM);
            oq.setLocalizerImuHeadingScalar(OQ_IMU_SCALAR);
            oq.setLocalizerVelocityIntervalMS(25);
            oq.resetLocalizer();

            // Save settings
            oq.saveParametersToFlash();
            initialized = true;
        }

        // Read all the desired data
        oq.readAllEncoderData(encoderDataBlock);

    }

    // Determine the distance from the back of robot to the perimeter wall (in inches).
    public double getBackRangeInches() {

        double range = (encoderDataBlock.positions[4] / 25.4) - 2;

        myOpMode.telemetry.addData("Sonar Range", "%4.2f in.", range);
        return (range);
    }

}
