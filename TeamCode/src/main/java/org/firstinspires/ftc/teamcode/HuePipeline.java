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

import java.util.Arrays;
import java.util.Comparator;

		/*
		Black: To create black in HSV:
		Set the value (V) to 0%. This plunges us into the abyss of darkness.
		Hue and saturation can be anything—they’re irrelevant when V is at rock bottom.
		White: Ah, the purity of light! To conjure white in HSV:
		Set the value (V) to 100%. We’re bathing in celestial radiance.
		Saturation (S) takes a backseat—it’s 0%. No vividness here.
		Hue can still be anything—it’s like the chameleon of colors.
		*/


public class HuePipeline implements VisionProcessor {

	// Private Members
	private	DetectedColor detectedColor = new DetectedColor(ColorSwatch.BLACK, 0,0,0);

	private Rect	window;
	private int		numPixels;

	//  These values give you more colors, but greater chance of the wrong color being detected.
	// private int[]   			colorHues  = { 0, 15, 23, 60, 90, 120, 135, 150};  // hues range from 0 - 180
	// private ColorSwatch[] 	colorNames = {ColorSwatch.RED, ColorSwatch.ORANGE, ColorSwatch.YELLOW,
	//							  		  	  ColorSwatch.GREEN, ColorSwatch.CYAN, ColorSwatch.BLUE,
	//									  	  ColorSwatch.VIOLET, ColorSwatch.MAGENTA};

	//  These values give you less colors, but smaller chance of the wrong color being detected.
	private int[]   		colorHues  = { 0, 23, 60, 120, 140};  // hues range from 0 - 180
	private ColorSwatch[] 	colorNames = {ColorSwatch.RED, ColorSwatch.YELLOW,
								  		  ColorSwatch.GREEN, ColorSwatch.BLUE,
										  ColorSwatch.PURPLE};


	private int K = 3; // Get the top 3 color hues
	private int	shortestHueDist		= 180;
	private int	shortestHueIndex	= 0;

	public HuePipeline(int X, int Y, int width, int height) {
		// set the Window of Interest   Michael: lts discuss how this should be defined for ease of use.
		setWindow(X, Y, width, height);
	}

	public void	setWindow(int X, int Y, int width, int height) {
		// current:  Center the window of the x:y position in pixels.
		X = Math.max(X - (width / 2), 0);
		Y = Math.max(Y - (height / 2), 0);
		window = new Rect(X, Y, width, height);
		numPixels = width * height;
	}

	@Override
	public void init(int width, int height, CameraCalibration calibration) {
	}
	
	@Override
	public Object processFrame(Mat rgbImage, long captureTimeNanos) {

		// Extracted channels
		Mat hueValues = new Mat();
		Mat satValues = new Mat();
		Mat valValues = new Mat();

		// select window of interest and convert to HSV space.
		Mat hsvWOI = new Mat();
		Imgproc.cvtColor(new Mat(rgbImage, window), hsvWOI, COLOR_RGB2HSV);

		Core.extractChannel(hsvWOI, hueValues, 0);  // Michael... could probably be done as a 2D single array...
		Core.extractChannel(hsvWOI, satValues, 1);
		Core.extractChannel(hsvWOI, valValues, 2);

		// Test for black & White first, to avoid more taxing KMEANS code.
		double avgValue = Core.sumElems(valValues).val[0] / numPixels;
		double avgSaturation = Core.sumElems(satValues).val[0] / numPixels;

		if (avgValue < 40) {
			detectedColor = new DetectedColor(ColorSwatch.BLACK,0, 0, 0);
		} else if ((avgSaturation < 40) && (avgValue > 100)) {
			detectedColor = new DetectedColor(ColorSwatch.WHITE,0, 0,	255 );
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
			int primeHue = clusterHue[0];

			// now scan the colorHue table to fin the table entry closest to the prime hue.
			shortestHueDist  = 180;
			shortestHueIndex = 0;

			for (int i = 0; i < colorHues.length; i++) {
				int length = Math.abs(primeHue - colorHues[i]);
				if (length > 90) {
					// wrap it around
					length = 180 - length;
				}
				if (length < shortestHueDist) {
					shortestHueDist = length;
					shortestHueIndex = i;
				}
			}

			detectedColor = new DetectedColor(colorNames[shortestHueIndex] ,clusterHue[0], 255,	255 );
		}

		return detectedColor;
	}


	@Override
	/**
	 * Draw a rectangle arounfd the current Window of interest.
	 * Make the color of the rectangle the same as the prime detected color (in HSV)
	 */
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

		DetectedColor detectedColor = (DetectedColor)userContext;

		float[] primeHSV = {(float)detectedColor.hsv()[0] * 2, (float)detectedColor.hsv()[1], detectedColor.hsv()[2]};
		Paint primeHuePaint = new Paint();
		primeHuePaint.setStyle(Paint.Style.STROKE);
		primeHuePaint.setStrokeWidth(scaleCanvasDensity * 8);
		primeHuePaint.setColor(android.graphics.Color.HSVToColor(primeHSV));
		canvas.drawRect(makeGraphicsRect(window, scaleBmpPxToCanvasPx), primeHuePaint);
	}

	public DetectedColor getDetectedColor() {
		return detectedColor;
	}

	private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
		int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
		int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
		int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
		int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

		return new android.graphics.Rect(left, top, right, bottom);
	}
}

