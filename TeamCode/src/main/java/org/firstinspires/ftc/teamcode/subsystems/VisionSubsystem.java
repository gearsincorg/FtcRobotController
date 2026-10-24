package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Webcam AprilTag Cluster tracking.
 *
 * Every scheduler loop (periodic) the latest detections are scanned for the cluster whose name
 * contains the requested target name (e.g. "RED" or "BLUE").  The pose of that cluster is then
 * available from getTargetPose(), which returns null if the cluster is not currently seen.
 *
 * Call setTargetName() or setAlliance() to choose which cluster to look for.
 */
public class VisionSubsystem extends SubsystemBase {
    private static final String CAMERA_NAME = "Webcam 1";
    private static final Size   CAMERA_SIZE = new Size(800, 600);

    // Low exposure time reduces motion blur.
    private static final int    EXPOSURE_MS = 6;
    private static final int    GAIN        = 250;

    // Ignore clusters that are seen from behind.
    private static final double MAX_ABS_ROLL = 90;

    private final OpMode opMode;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private String targetName = "RED";
    private AprilTagClusterDetection targetCluster = null;

    // Goal tip detection: how far the tag's pitch must change (degrees) to count as tipped.  Tune.
    public static double TIP_DELTA_DEG = 10;
    private double restPitch = Double.NaN;

    private int exposureStep = 0;  // 0 = waiting for camera, 1 = manual mode set, 2 = done

    // == standard Methods ===================================
    public VisionSubsystem(OpMode myOpMode) {
        opMode = myOpMode;
    }

    public void init() {
        aprilTag = new AprilTagProcessor.Builder()
            .setDrawCubeProjection(true)
            .build();

        visionPortal = new VisionPortal.Builder()
            .setCamera(opMode.hardwareMap.get(WebcamName.class, CAMERA_NAME))
            .setCameraResolution(CAMERA_SIZE)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build();
    }

    @Override
    public void periodic() {
        updateExposure();
        updateTarget();
        showTelemetry();
    }

    private void showTelemetry() {
        if (targetCluster != null) {
            AprilTagPoseFtc pose = targetCluster.ftcPose;
            opMode.telemetry.addData("VISION", "%s  %d%%  X %5.1f Y %5.1f  R %5.1f B %5.1f Yaw %5.1f",
                    targetCluster.metadata.name, targetCluster.percentClusterFound,
                    pose.x, pose.y, pose.range, pose.bearing, pose.yaw);
        } else {
            opMode.telemetry.addData("VISION", "no %s cluster", targetName);
        }
    }

    // == Target selection =========================================

    /** Look for the cluster whose name contains this text (case sensitive). */
    public void setTargetName(String name) {
        targetName = name;
    }

    /** Convenience for the "RED" / "BLUE" cluster names. */
    public void setAlliance(boolean areWeRed) {
        setTargetName(areWeRed ? "RED" : "BLUE");
    }

    public String getTargetName() {
        return targetName;
    }

    // == Pose data ================================================

    /** @return true if the desired cluster was seen on the most recent update. */
    public boolean hasTarget() {
        return targetCluster != null;
    }

    /**
     * @return the pose of the desired cluster relative to the camera (x right, y forward, z up,
     *         plus range, bearing, elevation, pitch, roll and yaw), or null if it is not visible.
     */
    public AprilTagPoseFtc getTargetPose() {
        return (targetCluster != null) ? targetCluster.ftcPose : null;
    }

    /** @return percent of the cluster's tags that are visible (0 if no target). */
    public int getTargetPercentFound() {
        return (targetCluster != null) ? targetCluster.percentClusterFound : 0;
    }

    // == Camera control ===========================================

    public void stopStreaming() {
        if (visionPortal != null) visionPortal.stopStreaming();
    }

    public void resumeStreaming() {
        if (visionPortal != null) visionPortal.resumeStreaming();
    }

    /** Release the camera.  Call when the OpMode is finished with vision. */
    public void close() {
        if (visionPortal != null) visionPortal.close();
    }

    private void updateTarget() {
        targetCluster = null;
        if (aprilTag == null) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.metadata.name.contains(targetName) &&
                    Math.abs(cluster.ftcPose.roll) < MAX_ABS_ROLL) {
                    targetCluster = cluster;
                    break;
                }
            }
        }
    }

    /**
     * Manual exposure can only be set once the camera is streaming.  Rather than blocking, do it
     * over successive periodic() calls: set Manual mode first, then exposure and gain.
     */
    private void updateExposure() {
        if (visionPortal == null || exposureStep >= 2) return;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        if (exposureStep == 0) {
            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
            }
            exposureStep = 1;
        } else {
            visionPortal.getCameraControl(ExposureControl.class)
                    .setExposure((long) EXPOSURE_MS, TimeUnit.MILLISECONDS);
            visionPortal.getCameraControl(GainControl.class).setGain(GAIN);
            exposureStep = 2;
        }
    }

    // == Command methods  ==========================================

    /**
     * Wait for the goal to tip up.  The first pitch seen is the resting pitch; this finishes
     * once the target's pitch has moved TIP_DELTA_DEG from it (either direction).  If the target
     * is lost the wait just continues, and the baseline is kept.  Add a timeout in the caller.
     */
    public Command waitForTipCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> restPitch = Double.NaN),
            new WaitUntilCommand(this::isTipped));
    }

    /** Same, but gives up after timeoutMs as a safety net (for auto). */
    public Command waitForTipCommand(long timeoutMs) {
        return waitForTipCommand().withTimeout(timeoutMs);
    }

    private boolean isTipped() {
        if (targetCluster == null) return false;
        double pitch = targetCluster.ftcPose.pitch;
        if (Double.isNaN(restPitch)) restPitch = pitch;
        return Math.abs(pitch - restPitch) >= TIP_DELTA_DEG;
    }

    public Command stopStreamingCommand() {
        return new InstantCommand(() -> stopStreaming());
    }

    public Command resumeStreamingCommand() {
        return new InstantCommand(() -> resumeStreaming());
    }
}
