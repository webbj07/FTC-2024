package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.teamcode.interfaces.PropDetector;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
public class PropDetectionProcessor implements VisionProcessor, PropDetector {
    // Alliance type and side passed to constructor
    private final ScrappySettings.AllianceType ALLIANCE_TYPE;
    private final ScrappySettings.AllianceSide ALLIANCE_SIDE;

    // Mats
    private final Mat testMat = new Mat();
    private final Mat lowMat = new Mat();
    private final Mat highMat = new Mat();
    private final Mat finalMat = new Mat();

    // Rectangles
    private final Rect LEFT_RECTANGLE;
    private final Rect RIGHT_RECTANGLE;

    // Red constants
    private final double RED_THRESHOLD = 0.3;
    private final Scalar LOW_HSV_RED_LOWER = new Scalar(0, 100, 20);
    private final Scalar LOW_HSV_RED_UPPER = new Scalar(10, 255, 255);
    private final Scalar HIGH_HSV_RED_LOWER = new Scalar(160, 100, 20);
    private final Scalar HIGH_HSV_RED_UPPER = new Scalar(180, 255, 255);

    // Blue constants
    private final double BLUE_THRESHOLD = 0.3;
    private final Scalar LOW_HSV_BLUE_LOWER = new Scalar(100, 100, 20);
    private final Scalar LOW_HSV_BLUE_UPPER = new Scalar(130, 255, 255);
    private final Scalar HIGH_HSV_BLUE_LOWER = new Scalar(220, 100, 20);
    private final Scalar HIGH_HSV_BLUE_UPPER = new Scalar(250, 255, 255);

    private DetectionResult detectionResult = DetectionResult.LEFT;

    public PropDetectionProcessor(ScrappySettings.AllianceType allianceType, ScrappySettings.AllianceSide allianceSide) {
        this.ALLIANCE_TYPE = allianceType;
        this.ALLIANCE_SIDE = allianceSide;

        if ((ALLIANCE_SIDE == ScrappySettings.AllianceSide.CLOSE && ALLIANCE_TYPE == ScrappySettings.AllianceType.BLUE) || (ALLIANCE_SIDE == ScrappySettings.AllianceSide.FAR && ALLIANCE_TYPE == ScrappySettings.AllianceType.RED)) {
            LEFT_RECTANGLE = new Rect(new Point(20, 80), new Point(150, 210));
            RIGHT_RECTANGLE = new Rect(new Point(385, 55), new Point(470, 160));
        } else {
            LEFT_RECTANGLE = new Rect(new Point(200, 65), new Point(300, 200));
            RIGHT_RECTANGLE = new Rect(new Point(505, 70), new Point(630, 210));
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        if (this.ALLIANCE_TYPE == ScrappySettings.AllianceType.RED) {
            Core.inRange(testMat, LOW_HSV_RED_LOWER, LOW_HSV_RED_UPPER, lowMat);
            Core.inRange(testMat, HIGH_HSV_RED_LOWER, HIGH_HSV_RED_UPPER, highMat);
        } else {
            Core.inRange(testMat, LOW_HSV_BLUE_LOWER, LOW_HSV_BLUE_UPPER, lowMat);
            Core.inRange(testMat, HIGH_HSV_BLUE_LOWER, HIGH_HSV_BLUE_UPPER, highMat);
        }

        testMat.release();

        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255;

        double allianceBasedThreshold = ALLIANCE_TYPE == ScrappySettings.AllianceType.RED ? RED_THRESHOLD : BLUE_THRESHOLD;

        if ((ALLIANCE_SIDE == ScrappySettings.AllianceSide.CLOSE && ALLIANCE_TYPE == ScrappySettings.AllianceType.BLUE) || (ALLIANCE_SIDE == ScrappySettings.AllianceSide.FAR && ALLIANCE_TYPE == ScrappySettings.AllianceType.RED)) {
            if (averagedLeftBox > allianceBasedThreshold) {
                detectionResult = DetectionResult.LEFT;
            } else if (averagedRightBox > allianceBasedThreshold){
                detectionResult = DetectionResult.MIDDLE;
            } else {
                detectionResult = DetectionResult.RIGHT;
            }
        } else {
            if (averagedLeftBox > allianceBasedThreshold) {
                detectionResult = DetectionResult.MIDDLE;
            } else if (averagedRightBox > allianceBasedThreshold){
                detectionResult = DetectionResult.RIGHT;
            } else {
                detectionResult = DetectionResult.LEFT;
            }
        }

        if (!ScrappySettings.IS_COMPETITION) {
            finalMat.copyTo(frame);
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(2);

        RectF leftRect = new RectF(LEFT_RECTANGLE.x * scaleBmpPxToCanvasPx, LEFT_RECTANGLE.y * scaleBmpPxToCanvasPx, (LEFT_RECTANGLE.x + LEFT_RECTANGLE.width) * scaleBmpPxToCanvasPx, (LEFT_RECTANGLE.y + LEFT_RECTANGLE.height) * scaleBmpPxToCanvasPx);
        canvas.drawRect(leftRect, paint);

        paint.setColor(Color.BLUE);
        RectF rightRect = new RectF(RIGHT_RECTANGLE.x * scaleBmpPxToCanvasPx, RIGHT_RECTANGLE.y * scaleBmpPxToCanvasPx, (RIGHT_RECTANGLE.x + RIGHT_RECTANGLE.width) * scaleBmpPxToCanvasPx, (RIGHT_RECTANGLE.y + RIGHT_RECTANGLE.height) * scaleBmpPxToCanvasPx);
        canvas.drawRect(rightRect, paint);
    }

    @Override
    public DetectionResult getDetectionResult() {
        return this.detectionResult;
    }
}