package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;

import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class ColorSensorProcessor implements VisionProcessor {

	// Private Members
	private SensedColor sensedColor = new SensedColor();
	private ColorWOI	colorWOI;
	private int			srcWidth;
	private int			srcHeight;
	private	Rect		window = new Rect();
	private Swatch[] 	swatches;

	private int 	K = 10; // Get the top n color hues
	private int		shortestHueDist		= 180;
	private	Swatch 	bestSwatch;

	public ColorSensorProcessor(ColorWOI colorWOI, Swatch[] swatches) {
		this.colorWOI = colorWOI;  // Set according to user request
		this.swatches = swatches;
	}

	/*
	 * Change the window of interest after the processor has been created.
	 */
	public void setWOI(ColorWOI colorWOI) {
		this.colorWOI = colorWOI;  // Set according to user request
		window = colorWOI.getOpenCVRect(srcWidth, srcHeight);
	}

	/*
	 * Change the window of interest after the processor has been created.
	 */
	public void	setSwatches(Swatch[] swatches) {
		this.swatches = swatches;
	}

	@Override
	public void init(int width, int height, CameraCalibration calibration) {
		srcWidth = width;
		srcHeight = height;
		window = colorWOI.getOpenCVRect(srcWidth, srcHeight);
	}
	
	@Override
	public Object processFrame(Mat rgbImage, long captureTimeNanos) {

		// Extracted channels
		Mat hueValues = new Mat();
		Mat satValues = new Mat();
		Mat valValues = new Mat();

		int srcPixels = window.width * window.height;

		// extract the window of interest and convert to HSV space.
		Mat myWOI = new Mat();
		Imgproc.cvtColor(new Mat(rgbImage, window), myWOI, COLOR_RGB2HSV);

		Core.extractChannel(myWOI, hueValues, 0);  // Michael... could probably be done as a 2D single array...
		Core.extractChannel(myWOI, satValues, 1);
		Core.extractChannel(myWOI, valValues, 2);

		// Test for black & White first, to avoid more taxing KMEANS code.
		int avgSaturation = (int)(Core.sumElems(satValues).val[0] / srcPixels);
		int avgValue 	  = (int)(Core.sumElems(valValues).val[0] / srcPixels);

		if (avgValue < 50) {
			sensedColor = new SensedColor(Swatch.BLACK, 0, avgSaturation, avgValue);
		} else if ((avgSaturation < 50) && (avgValue > 100)) {
			sensedColor = new SensedColor(Swatch.WHITE, 0, avgSaturation, avgValue);
		} else {

			// Reshape the hue values into a 1D array
			Mat reshapedHue = hueValues.reshape(1, hueValues.rows() * hueValues.cols());
			reshapedHue.convertTo(reshapedHue, CvType.CV_32F);

			// Perform K-Means clustering
			Mat labels = new Mat();
			Mat centers = new Mat(K, reshapedHue.cols(), reshapedHue.type());
			TermCriteria criteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 10, 2.0);
			Core.kmeans(reshapedHue, K, labels, criteria, 1, Core.KMEANS_PP_CENTERS, centers);

			Integer[] clusterCounts = new Integer[K];
			Integer[] clusterHue = new Integer[K];

			// Get the cluster centers (representing dominant hues), plus initialize the counts;
			for (int i = 0; i < K; i++) {
				clusterHue[i] = (int) centers.get(i, 0)[0];
				clusterCounts[i] = 0;
			}

			for (int i = 0; i < labels.rows(); i++) {
				int clusterIndex = (int) labels.get(i, 0)[0];
				clusterCounts[clusterIndex]++; //= clusterCounts[clusterIndex] + 1;
			}

			// Sort the data array using the custom comparator Get the cluster with the most entries.
			Comparator<Integer> customComparator = Comparator.comparingInt(color -> clusterCounts[Arrays.asList(clusterHue).indexOf(color)]);
			Arrays.sort(clusterHue, customComparator);
			Arrays.sort(clusterCounts);

			//Log.d("HUE ALL", showMat(reshapedHue));
			//Log.d("HUES TOP", showArray(clusterHue));
			//Log.d("HUES CNT", showArray(clusterCounts));

			// Grab the hue of the cluster with the most close colors...  this is the best hue match.
			int bestHue = clusterHue[K-1];

			// build a list of valid hues from the swatches, eliminate black and white
			List<Swatch> colorHues = new ArrayList<>();
			for (Swatch swatch : swatches){
				if (swatch.getHue() >= 0) {
					colorHues.add(swatch);
				}
			}

			// now scan the colorHue table to fin the table entry closest to the prime hue.
			shortestHueDist  = 180;
			bestSwatch = colorHues.get(0);

			for (Swatch swatch : colorHues) {
				int hueError = Math.abs(bestHue - swatch.getHue());
				if (hueError > 90) {
					// wrap it around
					hueError = 180 - hueError;
				}
				if (hueError < shortestHueDist) {
					shortestHueDist = hueError;
					bestSwatch = swatch;
				}
			}
			sensedColor = new SensedColor(bestSwatch, bestHue, avgSaturation, avgValue);
		}

		return sensedColor;
	}

	@Override
	/**
	 *  Draw a rectangle around the current Window of interest.
	 *  Make the color of the rectangle the same as the prime detected color (in HSV)
	 */
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

		float[] borderHSV;
		SensedColor sensedColor = (SensedColor)userContext;

		Swatch swatch = sensedColor.swatch();

		if (swatch == Swatch.BLACK) {
			borderHSV = new float[]{0, 0, 0};
		} else if (swatch == Swatch.WHITE) {
			borderHSV = new float[] {0, 0, 255};
		} else {
			borderHSV = new float[] {(float) sensedColor.hue() * 2, 255, 255};
		}

		Paint woiPaint = new Paint();
		woiPaint.setStyle(Paint.Style.STROKE);
		woiPaint.setStrokeWidth(scaleCanvasDensity * 8);
		woiPaint.setColor(android.graphics.Color.HSVToColor(borderHSV));
		canvas.drawRect(makeGraphicsRect(window, scaleBmpPxToCanvasPx), woiPaint);
	}

	public SensedColor getSensedColor() {
		return sensedColor;
	}

	private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
		int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
		int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
		int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
		int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

		return new android.graphics.Rect(left, top, right, bottom);
	}

	// ===  FOR Debugging Only...   ===============================================

	private String showMat(Mat stuff) {
		String result = "";
		for (int i = 0 ; i < stuff.rows(); i++) {
			result += String.format("%.0f ", stuff.get(i, 0)[0]);
		}
		return result;
	}

	private String showArray(Integer[] stuff) {
		String result = "";

		for (int i = 0 ; i < stuff.length; i++) {
			result += String.format("%d ", stuff[i]);
		}
		return result;
	}

}

