package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.BiobuzzRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
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
 *
 * Ball finder shares the same camera: a color
 * locator finds yellow blobs (BiobuzzRobot.POLLEN_YELLOW) anywhere in the image and picks the one
 * lowest in the image (the nearest).  getBallError() says how far it is off center, for steering
 * toward it; getBallHeadingErrorRad() turns that into a heading error (radians) for the heading controller.
 *
 * Both finders are OFF after init; an opmode turns on what it needs with enableAprilTags() /
 * enableBallFinder().  They are switched by a wrapper (Switchable), never with
 * VisionPortal.setProcessorEnabled(): a disabled color locator crashes the camera preview (the
 * SDK draws it with no results).
 */
public class VisionSubsystem extends SubsystemBase {
    private static final String CAMERA_NAME = "Webcam 1";
    // Same as the yellow test program (Concept: Color-Locator (Rectangle)): 320x240, auto exposure.
    private static final Size   CAMERA_SIZE = new Size(320, 240);

    // Manual exposure (low exposure time reduces motion blur, good for AprilTags) is off so the
    // colors match the test program.  Turning it on darkens the image: re-measure POLLEN_YELLOW.
    private static final boolean MANUAL_EXPOSURE = false;
    private static final int    EXPOSURE_MS = 6;
    private static final int    GAIN        = 250;

    // Ignore clusters that are seen from behind.
    private static final double MAX_ABS_ROLL = 90;
    private final double MAX_SPEED = 0.3;

    private final OpMode opMode;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private String targetName = "RED";
    private AprilTagClusterDetection targetCluster = null;

    // Goal tip detection: how far the tag's pitch must change (degrees) to count as tipped.  Tune.
    public static double TIP_DEG = 5;
    public static double TIP_THRESHOLD = 10;

    private int exposureStep = 0;  // 0 = waiting for camera, 1 = manual mode set, 2 = done

    // == Ball finder ==============================================
    // Only blobs in this size range (pixels) count, as in the test program.  Tune.
    public static int MIN_BALL_AREA = 400;
    public static int MAX_BALL_AREA = 6000;
    // Horizontal field of view of the camera at the CAMERA_SIZE resolution, degrees.  Turns the
    // ball's position in the image into a heading error.  Enter by hand (measure it: put a ball at
    // the image edge and measure the angle).  The default is a guess for a 4:3 webcam.
    public static double CAMERA_HFOV_DEG = 60;

    private ColorBlobLocatorProcessor ballLocator;
    private ColorBlobLocatorProcessor.Blob ball = null;

    // On/off wrappers around the two processors (both off after init).
    private Switchable aprilTagSwitch;
    private Switchable ballSwitch;

    // == standard Methods ===================================
    public VisionSubsystem(OpMode myOpMode) {
        opMode = myOpMode;
    }

    /** Opens the camera with both finders OFF.  Opmodes turn on what they use (enableAprilTags / enableBallFinder). */
    public void init() {
        aprilTag = new AprilTagProcessor.Builder()
            .setDrawCubeProjection(true)
            .build();

        // Same settings as the Concept: Color-Locator (Rectangle) test program.
        ballLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(BiobuzzRobot.POLLEN_YELLOW)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setErodeSize(3)
            .setDrawContours(true)   // the default box fit draws a rectangle around each blob
            .setBlurSize(5)
            .build();

        aprilTagSwitch = new Switchable(aprilTag);
        ballSwitch     = new Switchable(ballLocator);

        visionPortal = new VisionPortal.Builder()
            .setCamera(opMode.hardwareMap.get(WebcamName.class, CAMERA_NAME))
            .setCameraResolution(CAMERA_SIZE)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessors(aprilTagSwitch, ballSwitch)
            .build();
    }

    // == Enabling =================================================
    // Each finder costs camera-thread CPU (which slows the robot loop), so both start off and an
    // opmode turns on only what it needs, when it needs it.  Not VisionPortal.setProcessorEnabled():
    // a disabled color locator crashes the camera preview (the SDK draws it with no results).

    /** AprilTag cluster tracking on/off.  Off: hasTarget() is false. */
    public void enableAprilTags(boolean on)  { if (aprilTagSwitch != null) aprilTagSwitch.setEnabled(on); }
    /** Yellow ball finder on/off.  Off: hasBall() is false. */
    public void enableBallFinder(boolean on) { if (ballSwitch != null) ballSwitch.setEnabled(on); }
    public boolean isAprilTagsEnabled()  { return aprilTagSwitch != null && aprilTagSwitch.isEnabled(); }
    public boolean isBallFinderEnabled() { return ballSwitch != null && ballSwitch.isEnabled(); }

    /**
     * Wraps a processor so it can be switched off: while off, frames are not processed or drawn.
     * After switching on, results count only once a new frame has been processed, so the
     * processor's last (old) results are never used.
     */
    private static class Switchable implements VisionProcessor {
        private final VisionProcessor inner;
        private volatile boolean enabled = false;
        private volatile boolean fresh   = false;   // a frame has been processed since enabling

        Switchable(VisionProcessor inner) { this.inner = inner; }

        void setEnabled(boolean on) {
            if (on && !enabled) fresh = false;
            enabled = on;
        }
        boolean isEnabled()   { return enabled; }
        boolean hasResults()  { return enabled && fresh; }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            inner.init(width, height, calibration);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            if (!enabled) return null;
            Object ctx = inner.processFrame(frame, captureTimeNanos);
            fresh = true;
            return ctx;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            if (userContext == null) return;   // not processed (switched off)
            inner.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx,
                    scaleCanvasDensity, userContext);
        }
    }

    @Override
    public void periodic() {
        updateExposure();
        updateTarget();
        updateBall();
        showTelemetry();
    }

    private void showTelemetry() {
        // Each finder's line shows only while it is on.
        if (isAprilTagsEnabled()) {
            if (targetCluster != null) {
                AprilTagPoseFtc pose = targetCluster.ftcPose;
                opMode.telemetry.addData("Tags", "%s %d%%  x %4.1f y %4.1f  rng %4.1f brg %4.1f yaw %4.1f",
                        targetCluster.metadata.name, targetCluster.percentClusterFound,
                        pose.x, pose.y, pose.range, pose.bearing, pose.yaw);
            } else {
                opMode.telemetry.addData("Tags", "no %s cluster", targetName);
            }
        }
        if (isBallFinderEnabled()) {
            if (ball != null) {
                double error = getBallHeadingError();
                String side = Math.abs(error) < 0.1 ? "CENTER" : (error < 0 ? "LEFT" : "RIGHT");
                opMode.telemetry.addData("Ball", "%s  x %3.0f y %3.0f  err %5.2f  area %d",
                        side, ball.getBoxFit().center.x, ball.getBoxFit().center.y,
                        error, ball.getContourArea());
            } else {
                opMode.telemetry.addData("Ball", "none");
            }
        }
    }

    // == Target selection =========================================

    /** Convenience for the "RED" / "BLUE" cluster names. */
    public void setAlliance(boolean areWeRed) {
        targetName =  (areWeRed ? "RED" : "BLUE");
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

    // == Ball finder ==============================================

    /** @return true if the ball finder sees a ball. */
    public boolean hasBall() {
        return ball != null;
    }

    /**
     * @return how far the ball is from the image center, -1 (left edge) to +1 (right edge),
     *         or 0 if no ball is seen.
     */
    public double getBallHeadingError() {
        if (ball == null) return 0;
        double half = CAMERA_SIZE.getWidth() / 2.0;
        return (ball.getBoxFit().center.x - half) / half;
    }

    /**
     * @return how far the ball is from the image center, 0 (bottom) to +1 (top),
     *         or 0 if no ball is seen.
     */
    public double getBallForwardError() {
        if (ball == null) return 0;

        double distance = (CAMERA_SIZE.getHeight() - ball.getBoxFit().center.y) / CAMERA_SIZE.getHeight();
        return distance * MAX_SPEED;
    }

    /**
     * @return the heading error to the ball in radians (+ is counter-clockwise, like Pedro): the
     *         angle the robot must turn to point at it, or 0 if no ball is seen.  A ball to the
     *         right is a negative (clockwise) error.  Pinhole camera: the image edge is at
     *         +/- CAMERA_HFOV_DEG / 2.
     */
    public double getBallHeadingErrorRad() {
        if (ball == null) return 0;
        return -Math.atan(getBallHeadingError() * Math.tan(Math.toRadians(CAMERA_HFOV_DEG) / 2));
    }

    /**
     * @return the heading error to the target cluster in radians (+ is counter-clockwise), from the
     *         AprilTag bearing (degrees), or 0 if it is not visible.
     */
    public double getTargetHeadingErrorRad() {
        return (targetCluster == null) ? 0 : Math.toRadians(targetCluster.ftcPose.bearing);
    }

    private void updateBall() {
        ball = null;
        if (ballLocator == null || !ballSwitch.hasResults()) return;

        // Copy: getBlobs() is the SDK's own list, and filtering removes from it.
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>(ballLocator.getBlobs());
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, MIN_BALL_AREA, MAX_BALL_AREA, blobs);

        // The lowest blob in the image (largest y) is the nearest, so it wins.
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            if (ball == null || b.getBoxFit().center.y > ball.getBoxFit().center.y) ball = b;
        }
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
        if (aprilTag == null || !aprilTagSwitch.hasResults()) return;

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
        if (!MANUAL_EXPOSURE || visionPortal == null || exposureStep >= 2) return;
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
        // Turns AprilTags on for the wait and off again when it ends (tipped, timed out or cancelled).
        return new FunctionalCommand(
                () -> enableAprilTags(true),
                () -> {},
                interrupted -> enableAprilTags(false),
                this::isTipped);
    }

    /** Same, but gives up after timeoutMs as a safety net (for auto). */
    public Command waitForTipCommand(long timeoutMs) {
        return waitForTipCommand().withTimeout(timeoutMs);
    }

    private boolean isTipped() {
        if (targetCluster == null) return false;
        double pitch = targetCluster.ftcPose.pitch;
        return  Math.abs(pitch - TIP_DEG) <= TIP_THRESHOLD;
    }

    public Command stopStreamingCommand() {
        return new InstantCommand(() -> stopStreaming());
    }

    public Command resumeStreamingCommand() {
        return new InstantCommand(() -> resumeStreaming());
    }
}
