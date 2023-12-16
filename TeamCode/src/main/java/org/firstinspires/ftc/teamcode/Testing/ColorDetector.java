package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

// The detector needs to implement VisionProcessor to be be inserted into the Vision Portal and needs to implement CameraStreamSource to send images to the DS
 class ColorDetector implements VisionProcessor, CameraStreamSource {

    // The values for red and blue are only used by the detector
    private static final Scalar BLUE_HSV_HIGH = new Scalar(130, 255, 255);
    private static final Scalar BLUE_HSV_LOW = new Scalar(100, 50, 50);
    private static final Scalar RED1_HSV_HIGH = new Scalar(15, 255, 255);
    private static final Scalar RED1_HSV_LOW = new Scalar(0, 50, 50);
    private static final Scalar RED2_HSV_HIGH = new Scalar(180, 255, 255);
    private static final Scalar RED2_HSV_LOW = new Scalar(165, 50, 50);

    // These are used for drawing the debug boxes and are only used by the detector
    private static final Scalar RED = new Scalar(255,0,0);
    private static final Scalar GREEN = new Scalar(0,255,0);
    private static final Scalar BLUE = new Scalar(0,0,255);

    // The detection thresholds are only used by the detector and shouldn't change
    private static final double MIN_DETECTION_THRESH = 50;
    private static final int MIN_DETECTION_FRAMES = 30;

    // Telemetry doesn't need to be accessible and shouldn't change mid-run
    private final Telemetry telemetry;

    // viewMode and targetColor need to be accessible and changeable for debugging
    public ViewMode viewMode;
    public TargetColor targetColor;

    // We don't expect the zones to change and they don't need to be accessible outside the detector
    private final Rect leftZone;
    private final Rect midZone;

    // These enums need to be public so the robot code can use them
    public enum TargetColor {
        RED,
        BLUE,
    }

    public enum ViewMode {
        RAW,
        THRESHOLD,
    }

    public enum Detection {
        LEFT,
        MIDDLE,
        RIGHT,
        NONE,
    }

    // These are used each frame so the same variables are used to reduce memory pressure
    private final Mat rawFrame;
    private final Mat frameHSV;
    private final Mat red1Thresh;
    private final Mat red2Thresh;

    private final Mat thresh;

    private Mat leftFrame;
    private Mat midFrame;
    private Mat rightFrame;

    private double leftMean;
    private double midMean;
    private double rightMean;
    private double max;

    // The internal detection should not be accessible, getters can be used by robot code
    private Detection detection = Detection.NONE;

    // These should only be changed by the detector
    private int sameDetectionCount;
    private Detection previousDetection;
    private boolean confidentDetection;

    // Initialize all the values
    public ColorDetector(Telemetry telemetry, TargetColor targetColor, ViewMode viewMode, Rect leftZone, Rect midZone) {
        this.telemetry = telemetry;
        this.viewMode = viewMode;
        this.targetColor = targetColor;
        this.leftZone = leftZone;
        this.midZone = midZone;

        rawFrame = new Mat();
        frameHSV = new Mat();
        red1Thresh = new Mat();
        red2Thresh = new Mat();
        thresh = new Mat();
        leftFrame = new Mat();
        midFrame = new Mat();
        rightFrame = new Mat();
        leftMean = 0;
        midMean = 0;
        rightMean = 0;
        max = 0;
        sameDetectionCount = 0;
        previousDetection = Detection.NONE;
        confidentDetection = false;
    }

    // Should be used in robot code to determine what has been detected
    public Detection getConfidentDetection() {
        // Return the detection only if we are confident
        if (confidentDetection) {
            return detection;
        } else {
            return Detection.NONE;
        }
    }

    // Get detection regardless of confidence if that is needed for some reason
    public Detection getUnconfidentDetection() {
        return detection;
    }

    // Get current detection confidence
    public boolean isDetectionConfident() {
        return confidentDetection;
    }

    // All initialization is done in the constructor, we don't care when the Vision Portal initializes the detector
    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    // All the actual image processing is here
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Save raw frame
        frame.copyTo(rawFrame);

        // Convert to HSV for easier color finding
        Imgproc.cvtColor(frame, frameHSV, Imgproc.COLOR_RGB2HSV);

        switch (targetColor) {
            case BLUE:
                // Blue threshold
                Core.inRange(frameHSV, BLUE_HSV_LOW, BLUE_HSV_HIGH, thresh);
                break;
            case RED:
                // Red in HSV is at the top and bottom of H range so needs 2 thresholds that get combined
                Core.inRange(frameHSV, RED1_HSV_LOW, RED1_HSV_HIGH, red1Thresh);
                Core.inRange(frameHSV, RED2_HSV_LOW, RED2_HSV_HIGH, red2Thresh);
                Core.bitwise_or(red1Thresh, red2Thresh, thresh);
                break;
        }

        // Create submats for each detection zone
        leftFrame = thresh.submat(leftZone);
        midFrame = thresh.submat(midZone);

        // Find average brightness of each detection zone
        leftMean = Core.mean(leftFrame).val[0];
        midMean = Core.mean(midFrame).val[0];
        rightMean = Core.mean(rightFrame).val[0];

        // Find brightest detection zone and make sure it is above the minimum
        max = Math.max(leftMean, Math.max(midMean, rightMean));
        if (max > MIN_DETECTION_THRESH) {
            // Brightest zone is detection
            if (leftMean == max) {
                detection = Detection.LEFT;
            } else if (midMean == max) {
                detection = Detection.MIDDLE;
            } else if (rightMean == max) {
                detection = Detection.RIGHT;
            }

            // Detection is only confident if it has been consistent for multiple frames
            if (detection == previousDetection) {
                sameDetectionCount += 1;
                if (sameDetectionCount >= MIN_DETECTION_FRAMES) {
                    confidentDetection = true;
                }
            } else {
                confidentDetection = false;
                sameDetectionCount = 0;
            }

            // If no zone is above threshold, no detection is made
        } else {
            detection = Detection.NONE;
            confidentDetection = false;
            sameDetectionCount = 0;
        }

        // Remember detection for next frame's confidence counter
        previousDetection = detection;

        // Send telemetry to DS for debugging
        telemetry.addData("Target Color", targetColor.name());
        telemetry.addData("Detection", detection.name());
        telemetry.addData("Left", leftMean);
        telemetry.addData("Mid", midMean);
        telemetry.addData("Right", rightMean);
        telemetry.addData("Same Frames", sameDetectionCount);
        telemetry.addData("Confident", confidentDetection);
        telemetry.update();

        // This return value is used for passing state to onDrawFrame which we don't use so just return null
        return null;
    }

    // This is for drawing to the RC screen which we currently aren't doing so it does nothing
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

    // This is what sends debug images to the DS
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        // Display original or threshold based on viewMode
        Mat result = new Mat();
        switch (viewMode) {
            case RAW:
                rawFrame.copyTo(result);
                break;
            case THRESHOLD:
                Imgproc.cvtColor(thresh, result, Imgproc.COLOR_GRAY2RGB);
                break;
        }

        // Draw red rectangles around each detection zone
        Imgproc.rectangle(result, leftZone, RED);
        Imgproc.rectangle(result, midZone, RED);

        // Use blue if not confident, green if confident
        Scalar detectColor;
        if (confidentDetection) {
            detectColor = GREEN;
        } else {
            detectColor = BLUE;
        }

        // Draw blue/green rectangle around zone with detection
        switch (detection) {
            case LEFT:
                Imgproc.rectangle(result, leftZone, detectColor);
                break;
            case MIDDLE:
                Imgproc.rectangle(result, midZone, detectColor);
                break;
        }

        // Convert to bitmap and send to DS
        Bitmap resultBitmap = Bitmap.createBitmap(result.width(), result.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(result, resultBitmap);
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(resultBitmap));
    }

    // Helper function to create a Rect using center coordinates and dimensions
    public static Rect centerRect(int centerX, int centerY, int width, int height) {
        int x = centerX - width/2;
        int y = centerY - height/2;
        return new Rect(x, y, width, height);
    }
}