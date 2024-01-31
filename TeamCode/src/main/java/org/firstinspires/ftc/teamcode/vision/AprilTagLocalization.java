package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagLocalization {
    public static Vector2d getTagPosition(AprilTagDetection detection) {
        switch (detection.id) {
            case 1:
                return new Vector2d(60, 41.5);
            case 2:
                return new Vector2d(60, 35);
            case 3:
                return new Vector2d(60, 29.5);
            case 4:
                return new Vector2d(60, -29.5);
            case 5:
                return new Vector2d(60, -35);
            case 6:
                return new Vector2d(60, -41.5);
            default:
                return null;
        }
    }

    public static Vector2d getTagPosition(int detectionId) {
        switch (detectionId) {
            case 1:
                return new Vector2d(60, 41.5);
            case 2:
                return new Vector2d(60, 35);
            case 3:
                return new Vector2d(60, 29.5);
            case 4:
                return new Vector2d(60, -29.5);
            case 5:
                return new Vector2d(60, -35);
            case 6:
                return new Vector2d(60, -41.5);
            default:
                return null;
        }
    }

    public static Pose2d getRobotPositionFromTag(AprilTagDetection detection, double robotHeading, boolean isBackCamera) {
        double offset_x = isBackCamera ? ScrappySettings.BACK_CAMERA_OFFSET_X : ScrappySettings.FRONT_CAMERA_OFFSET_X;
        double offset_y = isBackCamera ? ScrappySettings.BACK_CAMERA_OFFSET_Y : ScrappySettings.FRONT_CAMERA_OFFSET_Y;

        double x = detection.ftcPose.x + offset_x;
        double y = detection.ftcPose.y + offset_y;

        double rotatedHeading = -robotHeading;

        double x2 = x * Math.cos(rotatedHeading) + y * Math.sin(rotatedHeading);
        double y2 = x * -Math.sin(rotatedHeading) + y * Math.cos(rotatedHeading);

        double absX;
        double absY;

        VectorF tagpose = detection.metadata.fieldPosition;
        if (detection.metadata.id <= 6) {
            absX = tagpose.get(0) + y2;
            absY = tagpose.get(1) - x2;
        } else {
            absX = tagpose.get(0) - y2;
            absY = tagpose.get(1) + x2;
        }

        return new Pose2d(absX, absY, robotHeading);
    }
}
