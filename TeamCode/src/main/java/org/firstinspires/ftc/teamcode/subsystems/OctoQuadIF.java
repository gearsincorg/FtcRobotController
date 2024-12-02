package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OctoQuadIF {

     // X_OFFSET_FROM_CENTER_MM is how sideways from the center of the robot is the X (forward) pod? Left increases
     // Y_OFFSET_FROM_CENTER_MM is how far forward from the center of the robot is the Y (Strafe) pod? forward increases
    static final float TICKS_PER_MM = 19.89f;
    static final float X_OFFSET_FROM_CENTER_MM = -27.0f;
    static final float Y_OFFSET_FROM_CENTER_MM =  25.8f;
    static final float OQ_IMU_SCALAR = 1.03f;
    static final int OQ_PORT_X = 0;
    static final int OQ_PORT_Y = 1;
    static final int SONAR_PORT = 4;

    private OctoQuad_v3         oq;
    private OctoQuad_v3.EncoderDataBlock encoderDataBlock = new OctoQuad_v3.EncoderDataBlock();
    private LinearOpMode        myOpMode;
    private boolean             showTelemetry = false;
    private double              range = 0;

    // Arm Constructor
    public OctoQuadIF(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void initialize(boolean showTelemetry) {
        // Connect to the OctoQuad by looking up its name in the hardwareMap.
        // Clear out all prior settings and encoder data before setting up desired configuration
        oq = myOpMode.hardwareMap.get(OctoQuad_v3.class, "octoquad");
        oq.resetEverything();

        // Set the first 4 channels as relative encoders and the next 4 as absolute encoders
        oq.setChannelBankConfig(OctoQuad_v3.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);

        // Configure the localizer
        oq.setSingleEncoderDirection(OQ_PORT_X, OctoQuadBase_v3.EncoderDirection.REVERSE);
        oq.setSingleEncoderDirection(OQ_PORT_Y, OctoQuadBase_v3.EncoderDirection.REVERSE);

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
        this.showTelemetry = showTelemetry;
    }

    public void update() {

        // Read all the desired data
        oq.readAllEncoderData(encoderDataBlock);
        range = (encoderDataBlock.positions[SONAR_PORT] / 25.4);

        OctoQuad_v3.LocalizerDataBlock localizer = new OctoQuad_v3.LocalizerDataBlock();
        if (showTelemetry){
            myOpMode.telemetry.addData("Sonar Range", "%4.2f in.", range);
            oq.readLocalizerData(localizer);

            myOpMode.telemetry.addData("Localizer status", localizer.localizerStatus);
            myOpMode.telemetry.addData("Heading deg", localizer.heading_rad*180/Math.PI);
            myOpMode.telemetry.addData("Heading dps", localizer.velHeading_radS*180/Math.PI);
            myOpMode.telemetry.addData("X mm", localizer.posX_mm);
            myOpMode.telemetry.addData("Y mm", localizer.posY_mm);
            myOpMode.telemetry.addData("VX mm/s", localizer.velX_mmS);
            myOpMode.telemetry.addData("VY mm/s", localizer.velY_mmS);
        }
    }

    // Determine the distance from the back of robot to the perimeter wall (in inches).
    public double getBackRangeInches() {
        return (range);
    }
}
