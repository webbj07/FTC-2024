package org.firstinspires.ftc.team4100worlds.interfaces;

public interface PropDetector {
    enum DetectionResult {
        LEFT, MIDDLE, RIGHT
    }

    DetectionResult getDetectionResult();
}
