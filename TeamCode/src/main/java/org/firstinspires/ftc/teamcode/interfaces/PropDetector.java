package org.firstinspires.ftc.teamcode.interfaces;

public interface PropDetector {
    enum DetectionResult {
        LEFT, MIDDLE, RIGHT
    }

    DetectionResult getDetectionResult();
}
