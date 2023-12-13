package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class TeamPropPipeline implements VisionProcessor {

	/**
	 * Uses HSVs for the scalars
	 */
	private final Scalar lowerHSV = new Scalar(0,   180,   0); // the lower hsv threshold for your detection
	private final Scalar upperHSV = new Scalar(180, 255, 255); // the upper hsv threshold for your detection

	private final double MIN_AREA   = 1000; // the minimum area for the detection to consider for your prop
	private final double LEFT_LINE  = 213 ;
	private final double RIGHT_LINE = 426 ;

	// Private Members
	private boolean allianceIsBlue = false;
	private TeamPropLocation teamPropLocation = TeamPropLocation.UNKNOWN;

	private boolean isBlue = false;
	private Paint linePaint;
	private final Mat hierarchy = new Mat();
	private int countoursFound = 0;

	private String targetString = new String();

	public TeamPropPipeline() {
		// setting up the paint for the lines that comprise the box
		linePaint = new Paint();
		linePaint.setColor(Color.GREEN); // you may want to change this
		linePaint.setAntiAlias(true);
		linePaint.setStrokeWidth(10); // or this
		linePaint.setStrokeCap(Paint.Cap.ROUND);
		linePaint.setStrokeJoin(Paint.Join.ROUND);
	}

	@Override
	public void init(int width, int height, CameraCalibration calibration) {
		// this method comes with all VisionProcessors, we just don't need to do anything here, and you dont need to call it
	}
	
	@Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		// this converts the frame from RGB to HSV, which is supposed to be better for doing colour blob detection

		Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
		Imgproc.GaussianBlur(frame, frame, new Size(23, 23), 3.6);
		Core.inRange(frame, lowerHSV, upperHSV, frame);

		ArrayList<MatOfPoint> contours = new ArrayList<>();
		Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		List<Rect> detectedRects = new ArrayList<>();

		int	contourCount = 0;
		double maxArea = 0;
		Rect maxBox = new Rect();

		for (MatOfPoint contour : contours) {
			contourCount++;
			Rect box = Imgproc.boundingRect(contour);

			// Only process reasonable shapes.
			if ((box.area() > MIN_AREA) && (box.width < 100) && (box.height < 100)) {
				detectedRects.add(box);

				// choose biggest box
				if (box.area() > maxArea) {
					maxArea = box.area();
					maxBox = box;
				}
			}

			// find out where the biggest target is.
			if (maxArea > 0) {
				double x = maxBox.x;
				if (x < LEFT_LINE) {
					teamPropLocation = TeamPropLocation.LEFT_SIDE;
				} else if (x > RIGHT_LINE) {
					teamPropLocation = TeamPropLocation.RIGHT_SIDE;
				} else {
					teamPropLocation = TeamPropLocation.CENTER;
				}
			} else {
				teamPropLocation = TeamPropLocation.UNKNOWN;
			}
		}

		countoursFound = contourCount;

		targetString = detectedRects.toString();
		return detectedRects;
	}
	
	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
		Paint redPaint = new Paint();
		redPaint.setColor(Color.RED);
		redPaint.setStyle(Paint.Style.STROKE);
		redPaint.setStrokeWidth(scaleCanvasDensity * 2);

		Paint greenPaint = new Paint();
		greenPaint.setColor(Color.GREEN);
		greenPaint.setStyle(Paint.Style.STROKE);
		greenPaint.setStrokeWidth(scaleCanvasDensity * 4);

		if (userContext != null) {
			List<Rect> detectedRects = (List<Rect>) userContext;
			for (Rect arect : detectedRects) {
				canvas.drawRect(makeGraphicsRect(arect, scaleBmpPxToCanvasPx), greenPaint);
			}
		}
	}

	public void close() {
		hierarchy.release();
	}

	public String getTargetString() {
		return targetString;
	}

	public int	getContourCount() {
		return countoursFound;
	}

	public TeamPropLocation getTeamPropLocation() {
		return teamPropLocation;
	}

	private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
		int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
		int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
		int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
		int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

		return new android.graphics.Rect(left, top, right, bottom);
	}
}
