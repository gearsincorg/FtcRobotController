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

	private final double MIN_AREA   = 1000; // the minimum area for the detection
	private final int MIN_WIDTH     = 20;  //
	private final int MIN_HEIGHT    = 20;  //

	private final int MAX_WIDTH     = 120; //
	private final int MAX_HEIGHT    = 120; //
	private final int TOP_OF_TARGET = 170 ;  // box's top must be greater than this value (lower in frame)
	private final int BOTTOM_OF_PERIMETER = 200 ;  // box's bottom must be greater than this value (lower in frame)
	private final Rect   LEFT_TARGET = new Rect(125,221,1,1);
	private final Rect   CENTER_TARGET = new Rect(318,211,1,1);
	private final Rect   RIGHT_TARGET = new Rect(519,217,1,1);

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
		List<Rect> validRects    = new ArrayList<>();

		int	contourCount = 0;
		double maxArea = 0;
		Rect maxBox = new Rect();

		for (MatOfPoint contour : contours) {
			contourCount++;
			Rect box = Imgproc.boundingRect(contour);

			if ((box.width > MIN_WIDTH) && (box.height > MIN_HEIGHT) && (box.area() > MIN_AREA)) {
				detectedRects.add(box);

				// Mark reasonable shapes. (low, correct size, in general correct area.)
				if ((box.y > TOP_OF_TARGET) &&
					((box.y + box.height) > BOTTOM_OF_PERIMETER) &&
					(box.width < MAX_WIDTH) &&
					(box.height < MAX_HEIGHT)
				) {
					double aspect = (double)box.width / (double)box.height;
					if ((aspect > 0.5) && (aspect < 1.5)) {
						validRects.add(box);
					}
				}
			}
		}

		// find out which target we want
		if (validRects.size() > 0) {
			// only one good target match
			teamPropLocation = decideLocation(validRects);
		} else if (detectedRects.size() > 0) {
			teamPropLocation = decideLocation(detectedRects);
		} else {
			teamPropLocation = TeamPropLocation.UNKNOWN;
		}

		countoursFound = contourCount;

		targetString = detectedRects.toString();
		return new Targets(detectedRects,validRects);
	}

	private TeamPropLocation decideLocation(List<Rect> rects) {

		TeamPropLocation location = TeamPropLocation.UNKNOWN;
		double minRange = 10000;
		double range 	;

		// check the distance from each of the three known target locations
		// Find the closest match
		for (Rect rect : rects) {
			range = getRange(rect, LEFT_TARGET);
			if (range < minRange) {
				minRange = range;
				location = TeamPropLocation.LEFT_SIDE;
			}

			range = getRange(rect, CENTER_TARGET);
			if (range < minRange) {
				minRange = range;
				location = TeamPropLocation.CENTER;
			}

			range = getRange(rect, RIGHT_TARGET);
			if (range < minRange) {
				minRange = range;
				location = TeamPropLocation.RIGHT_SIDE;
			}
		}

		return location;
	}

	private double getRange(Rect box, Rect target) {
		return(Math.hypot(target.x - (box.x + (box.width / 2)), target.y - (box.y + (box.height / 2)) ));
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
			Targets targets = (Targets)userContext;
			for (Rect arect : targets.detectedRects) {
				canvas.drawRect(makeGraphicsRect(arect, scaleBmpPxToCanvasPx), redPaint);
			}
			for (Rect arect : targets.validRects) {
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

class Targets {

	public List<Rect> detectedRects;
	public List<Rect> validRects;

	public Targets(List<Rect> detected, List<Rect> valid ){
		detectedRects = detected;
		validRects = valid;
	}
}
