package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2YCrCb;

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
	private YCrCb_Range yCrCb_Range;
	private int			srcWidth;
	private int			srcHeight;
	private	Rect		window = new Rect();
	private	ArrayList<Rect> foundBlobs = new ArrayList<>();

	public ColorFinderProcessor(ColorWOI colorWOI, YCrCb_Range yCrCb_Range) {
		this.colorWOI   = colorWOI;  // Set according to user request
		this.yCrCb_Range = yCrCb_Range;
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
	public void setFindColor(YCrCb_Range yCrCb_Range) {
		this.yCrCb_Range = yCrCb_Range;
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
		Imgproc.cvtColor(new Mat(rgbImage, window), myWOI, COLOR_RGB2YCrCb);

		// blur the image and then filter for the requested hue range.
		int radius = 1;
		int kernelSize = 6 * radius + 1;
		Imgproc.GaussianBlur(myWOI, myWOI, new Size(kernelSize, kernelSize), radius);

		// Log.d("MAT", showMat(myWOI));

		mask   = new Mat();
		Core.inRange(myWOI, yCrCb_Range.loRange, yCrCb_Range.hiRange, mask);

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

//		return filteredBlobs;
		return contours;
	}

	@Override
	/**
	 *  Draw a rectangle around the current Window of interest.
	 *  Make the color of the rectangle the same as the detection color (in HSV)
	 */
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

		float[] black   = new float[] {0, 0, 0};
		float[] white   = new float[] {0, 0, 255};
		float[] magenta = new float[] {300, 255, 255};

		// foundBlobs = (ArrayList<Rect>)userContext;

		List<MatOfPoint> contours = (List<MatOfPoint>)userContext;

		Paint woiPaint = new Paint();
		woiPaint.setStyle(Paint.Style.STROKE);
		woiPaint.setStrokeWidth(scaleCanvasDensity * 8);
		woiPaint.setColor(android.graphics.Color.HSVToColor(white));
		canvas.drawRect(makeGraphicsRect(window, scaleBmpPxToCanvasPx), woiPaint);

		Paint blobPaint = new Paint();
		blobPaint.setStyle(Paint.Style.STROKE);
		blobPaint.setStrokeWidth(scaleCanvasDensity * 4);
		blobPaint.setColor(android.graphics.Color.HSVToColor(magenta));

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
			blobPaint.setColor(android.graphics.Color.HSVToColor(black));
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

	private String showMat(Mat stuff) {
		String result = "";
		for (int r = 0 ; r < stuff.rows(); r++) {
			for (int c = 0 ; c < stuff.cols(); c++) {
				double[] pixel = stuff.get(r, c);
				result += String.format("(%.0f, %.0f, %.0f) ", pixel[0], pixel[1], pixel[2]);
			}
		}
		return result;
	}

}

