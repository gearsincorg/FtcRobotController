package org.firstinspires.ftc.teamcode.eocv;

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

public class CenterStagePipeline implements VisionProcessor {
//	Scalar lowerHSV = new Scalar(0, 200, 100); // the lower hsv threshold for your detection
//	Scalar upperHSV = new Scalar(30, 255, 255); // the upper hsv threshold for your detection

	Scalar lowerHSV = new Scalar(0,   180, 110); // the lower hsv threshold for your detection
	Scalar upperHSV = new Scalar(180, 255, 255); // the upper hsv threshold for your detection

	Scalar lowerHSV2 = new Scalar(150, 200, 100); // the lower hsv threshold for your detection
	Scalar upperHSV2 = new Scalar(180, 255, 255); // the upper hsv threshold for your detection


	Scalar lowerRGB = new Scalar(110, 0 , 0); // the lower hsv threshold for your detection
	Scalar upperRGB = new Scalar(255, 60, 74); // the upper hsv threshold for your detection
	double minArea = 100; // the minimum area for the detection to consider for your prop
	double leftLine = 213 ;
	double rightLine = 426 ;
	private final Paint linePaint;
	private final Mat hierarchy = new Mat();
	private MatOfPoint largestContour;

	/**
	 * Uses HSVs for the scalars
	 *
	 */
	public CenterStagePipeline() {
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

		Mat lobits = new Mat();
		Mat hibits = new Mat();

		Core.inRange(frame, lowerHSV, upperHSV, lobits);
		Core.inRange(frame, lowerHSV2, upperHSV2, hibits);

		Core.bitwise_or(lobits, hibits, frame);

// 		Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2RGB);
//		Core.inRange(frame, lowerRGB, upperRGB, frame);


		ArrayList<MatOfPoint> contours = new ArrayList<>();
		Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		List<Rect> detectedRects = new ArrayList<>();

		for (MatOfPoint contour : contours) {
			if (Imgproc.contourArea(contour) > minArea) {
					Rect box = Imgproc.boundingRect(contour);
					detectedRects.add(box);
			}
		}
		
		return detectedRects;
	}
	
	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
		Rect rect = new Rect(20, 20, 50, 50);

		Paint rectPaint = new Paint();
		rectPaint.setColor(Color.RED);
		rectPaint.setStyle(Paint.Style.STROKE);
		rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

		canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
		if (userContext != null) {
			List<Rect> detectedRects = (List<Rect>) userContext;
			for (Rect arect : detectedRects) {
				canvas.drawRect(makeGraphicsRect(arect, scaleBmpPxToCanvasPx), rectPaint);
			}
		}

	}

	public void close() {
		hierarchy.release();
	}

	private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
		int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
		int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
		int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
		int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

		return new android.graphics.Rect(left, top, right, bottom);
	}
}
