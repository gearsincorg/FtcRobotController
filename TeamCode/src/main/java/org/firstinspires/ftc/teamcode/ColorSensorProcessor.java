package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2YCrCb;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ColorSensorProcessor implements VisionProcessor {

	// Private Members
	private SensedColor sensedColor = new SensedColor();
	private ColorWOI	colorWOI;
	private int			srcWidth;
	private int			srcHeight;
	private	Rect		window = new Rect();
	private Swatch[] 	swatches;

	private int 	K = 5; // Get the top n color hues
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
		colorWOI.setSrc(srcWidth, srcHeight);
		window = colorWOI.getOpenCVRect();
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
		colorWOI.setSrc(srcWidth, srcHeight);
		window = colorWOI.getOpenCVRect();
	}
	
	@Override
	public Object processFrame(Mat rgbImage, long captureTimeNanos) {

		int srcPixels = window.width * window.height;

		// extract the window of interest and convert to HSV space.
		Mat myWOI = new Mat();
		Imgproc.cvtColor(new Mat(rgbImage, window), myWOI, COLOR_RGB2YCrCb);

		// Extract Y, Cr and Cb channels
		List<Mat> channels = new ArrayList<>();
		Core.split(myWOI, channels);
		Mat lChannel = channels.get(0);
		Mat crChannel = channels.get(1);
		Mat cbChannel = channels.get(2);

		// Determine the average Luminance
		int avgLuminance 	  = (int)(Core.sumElems(lChannel).val[0] / srcPixels);

		// Flatten data for K-means
		Mat data = new Mat(srcPixels, 2, CvType.CV_32F);
		for (int i = 0; i < crChannel.rows(); i++) {
			for (int j = 0; j < crChannel.cols(); j++) {
				data.put(i * crChannel.cols() + j, 0, crChannel.get(i, j)[0]);
				data.put(i * crChannel.cols() + j, 1, cbChannel.get(i, j)[0]);
			}
		}

		// Perform K-Means clustering
		Mat labels = new Mat();
		Mat centers = new Mat(K, data.cols(), data.type());
		TermCriteria criteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 10, 2.0);

		Core.kmeans(data, K, labels, criteria, 1, Core.KMEANS_PP_CENTERS, centers);

		Integer[] clusterCounts = new Integer[K];
		int maxCount = 0;
		int maxCountIndex = 0;

		// Get the number of pixels for each cluster
		for (int i = 0; i < K; i++) {
			clusterCounts[i] = 0;
		}

		// Get the bigest count along the way
		for (int i = 0; i < labels.rows(); i++) {
			int clusterIndex = (int) labels.get(i, 0)[0];
			int newCount = clusterCounts[clusterIndex]++;

			if (newCount > maxCount) {
				maxCount = newCount;
				maxCountIndex = clusterIndex;
			}
		}

		double Y  = avgLuminance; // Luminance
		double Cr = centers.get(maxCountIndex, 0)[0]; // Red-difference Chroma
		double Cb = centers.get(maxCountIndex, 1)[0]; // Blue-difference Chroma

		Log.d("BEST YCrCb", String.format("Y:%.0f, Cr:%.0f, Cb:%.0f", Y, Cr, Cb));

		Scalar yCrCb = new Scalar(Y, Cr, Cb);
		Scalar hue   = Spectrum.yCrCb2hsv(yCrCb);

		int H = (int)hue.val[0];
		int S = (int)hue.val[1];
		int V = (int)hue.val[2];

		Log.d("Best HSV", String.format("H:%d, S:%d, V:%d", H, S, V));

		// Check for Black or White before matching Hue.
		if ((S < 50) && (V > 180)) {
			sensedColor = new SensedColor(Swatch.WHITE, H, S, V);
		} else if ((S < 50) || (V < 50)) {
			sensedColor = new SensedColor(Swatch.BLACK, H, S, V);
		} else {

			// build a list of valid hues from the swatches, eliminate black and white
			List<Swatch> colorHues = new ArrayList<>();
			for (Swatch swatch : swatches) {
				if (swatch.getHue() >= 0) {
					colorHues.add(swatch);
				}
			}

			// now scan the colorHue table to find the table entry closest to the prime hue.
			// watch for hue wrap around at 180.
			shortestHueDist = 180;
			bestSwatch = colorHues.get(0);

			for (Swatch swatch : colorHues) {
				int hueError = Math.abs((int) H - swatch.getHue());
				if (hueError > 90) {
					// wrap it around
					hueError = 180 - hueError;
				}
				if (hueError < shortestHueDist) {
					shortestHueDist = hueError;
					bestSwatch = swatch;
				}
			}
			sensedColor = new SensedColor(bestSwatch, H, S, V);
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
			borderHSV = new float[] {(float) sensedColor.hue() * 2, sensedColor.sat(), sensedColor.val()};
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
			result += String.format("[%.0f %.0f] ", stuff.get(i, 0)[0], stuff.get(i, 1)[0]);
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

