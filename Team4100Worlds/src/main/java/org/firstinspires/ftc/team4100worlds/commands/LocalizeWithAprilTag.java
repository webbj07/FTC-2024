package org.firstinspires.ftc.team4100worlds.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100worlds.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.team4100worlds.vision.AprilTagLocalization;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class LocalizeWithAprilTag extends CommandBase {
    private final ScrappyAutoBase m_base;
    private final boolean m_isBackCamera;
    private boolean m_isFinished = false;

    public LocalizeWithAprilTag(ScrappyAutoBase base, boolean isBackCamera) {
        m_base = base;
        m_isBackCamera = isBackCamera;
    }

    @Override
    public void initialize() {
        double robotHeading = m_base.robot.m_drive.getPose().getHeading();

        double avgX = 0;
        double avgY = 0;
        int detections = 0;

        for (int i = 0; i < 3; i++) {
            List<AprilTagDetection> detectionList = m_base.getAprilTagDetections(m_isBackCamera);
            for (AprilTagDetection detection : detectionList) {
                Pose2d estimatedPos = AprilTagLocalization.getRobotPositionFromTag(detection, robotHeading, m_isBackCamera);
                avgX += estimatedPos.getX();
                avgY += estimatedPos.getY();
                detections++;
                break;
            }
        }

        if (detections != 0) {
            avgX /= detections;
            avgY /= detections;
            m_base.robot.m_drive.poseUpdater.setCurrentPoseUsingOffset(new Pose2d(avgX, avgY, robotHeading));
        }

        m_isFinished = true;
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
