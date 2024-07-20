package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class ColorFinderProcessor implements VisionProcessor {

	// Private Members
	private ColorWOI	colorWOI;
	private	HueRange	hueRange;
	private int			srcWidth;
	private int			srcHeight;
	private	Rect		window = new Rect();
	private	ArrayList<Rect> foundBlobs = new ArrayList<>();

	private static final int LOWER_SAT = 100;  // 110
	private static final int UPPER_SAT = 255;
	private static final int LOWER_VAL = 100;  // 110
	private static final int UPPER_VAL = 255;

	public ColorFinderProcessor(ColorWOI colorWOI, HueRange hueRange) {
		this.colorWOI = colorWOI;  // Set according to user request
		this.hueRange = hueRange;
	}

	/*
	 * Change the window of interest after the processor has been created.
	 */
	public void	setWOI(ColorWOI colorWOI) {
		this.colorWOI = colorWOI;  // Set according to user request
		window = colorWOI.getOpenCVRect(srcWidth, srcHeight);
	}

	/*
	 * Change the window of interest after the processor has been created.
	 */
	public void	setHueRange(HueRange hueRange) {
		this.hueRange = hueRange;
	}

	@Override
	public void init(int width, int height, CameraCalibration calibration) {
		srcWidth = width;
		srcHeight = height;
		window = colorWOI.getOpenCVRect(srcWidth, srcHeight);

	}
	
	@Override
	public Object processFrame(Mat rgbImage, long captureTimeNanos) {

		Mat hierarchy 	= new Mat();
		Mat myWOI 		= new Mat();
		Mat mask   		= new Mat();
		Scalar lowerHSV; // the lower hsv threshold
		Scalar upperHSV; // the upper hsv threshold


		// extract the window of interest and convert to HSV space.
		Imgproc.cvtColor(new Mat(rgbImage, window), myWOI, COLOR_RGB2HSV);

		// blur the image and then filter for the requested hue range.
		int radius = 5;
		int kernelSize = 6 * radius + 1;
		Imgproc.GaussianBlur(myWOI, myWOI, new Size(kernelSize, kernelSize), radius);

		if (hueRange.split()) {
			lowerHSV = new Scalar(0, LOWER_SAT, LOWER_VAL); // the lower hsv threshold
			upperHSV = new Scalar(hueRange.min(), UPPER_SAT, UPPER_VAL); // the upper hsv threshold
			Mat loMask   = new Mat();
			Core.inRange(myWOI, lowerHSV, upperHSV, loMask);

			lowerHSV = new Scalar(hueRange.max(), LOWER_SAT, LOWER_VAL); // the lower hsv threshold
			upperHSV = new Scalar(180, UPPER_SAT, UPPER_VAL); // the upper hsv threshold
			Mat hiMask   = new Mat();
			Core.inRange(myWOI, lowerHSV, upperHSV, hiMask);

			Core.bitwise_or(loMask, hiMask, mask);
		} else {
			lowerHSV = new Scalar(hueRange.min(), LOWER_SAT, LOWER_VAL); // the lower hsv threshold
			upperHSV = new Scalar(hueRange.max(), UPPER_SAT, UPPER_VAL); // the upper hsv threshold
			Log.d("LOHSV", lowerHSV.toString());
			Log.d("UpHSV", upperHSV.toString());
			Core.inRange(myWOI, lowerHSV, upperHSV, mask);
		}

		ArrayList<MatOfPoint> contours = new ArrayList<>();
		ArrayList<Rect> filteredBlobs  = new ArrayList<>();

		Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		// Sort contours by area in descending order
		contours.sort(new Comparator<MatOfPoint>() {
			public int compare(MatOfPoint c1, MatOfPoint c2) {
				return (int) (Imgproc.contourArea(c2) - Imgproc.contourArea(c1));
			}
		});

		// filter out the blobs we want...
		String areas = "";
		for (MatOfPoint contour : contours) {

			areas += String.format("%5.0f ", Imgproc.contourArea(contour));

			Rect   box = Imgproc.boundingRect(contour);
			filteredBlobs.add(box);
		}
		Log.d("CONTOUR AREAS", areas);

		//		Collections.sort(rectangles, Comparator.comparingDouble(Imgproc.contourArea(contour));
		// Sort the Blobs

//		return filteredBlobs;
		return contours;
	}

	@Override
	/**
	 *  Draw a rectangle around the current Window of interest.
	 *  Make the color of the rectangle the same as the detection color (in HSV)
	 */
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

		float[] borderHSV   = new float[] {0, 0, 255};
		float[] blobHSV     = new float[] {(float) hueRange.center() * 2, 255, 255};

		// foundBlobs = (ArrayList<Rect>)userContext;

		List<MatOfPoint> contours = (List<MatOfPoint>)userContext;

		Paint woiPaint = new Paint();
		woiPaint.setStyle(Paint.Style.STROKE);
		woiPaint.setStrokeWidth(scaleCanvasDensity * 8);
		woiPaint.setColor(android.graphics.Color.HSVToColor(borderHSV));
		canvas.drawRect(makeGraphicsRect(window, scaleBmpPxToCanvasPx), woiPaint);

		Paint blobPaint = new Paint();
		blobPaint.setStyle(Paint.Style.STROKE);
		blobPaint.setStrokeWidth(scaleCanvasDensity * 4);
		blobPaint.setColor(android.graphics.Color.HSVToColor(blobHSV));

		// Draw contours on the canvas
		for (MatOfPoint contour : contours) {
			Point[] points = contour.toArray();
			Path path = new Path();

			path.moveTo((float) (points[0].x  + window.x) * scaleBmpPxToCanvasPx, (float)(points[0].y + window.y) * scaleBmpPxToCanvasPx);
			for (int i = 1; i < points.length; i++) {
				path.lineTo((float) (points[i].x + window.x) * scaleBmpPxToCanvasPx, (float) (points[i].y +window.y) * scaleBmpPxToCanvasPx);
			}
			path.close(); // Close the contour path

			canvas.drawPath(path, blobPaint);
		}

		/*
		for (Rect blob : foundBlobs) {
			blob.x += window.x;
			blob.y += window.y;

			canvas.drawRect(makeGraphicsRect(blob, scaleBmpPxToCanvasPx), blobPaint);

		}
		*/
	}

	public List<Rect> getFoundBlobs() {
		return foundBlobs;
	}

	private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
		int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
		int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
		int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
		int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

		return new android.graphics.Rect(left, top, right, bottom);
	}
}

