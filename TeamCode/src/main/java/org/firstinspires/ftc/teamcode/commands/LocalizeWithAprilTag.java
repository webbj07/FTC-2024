package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalization;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class LocalizeWithAprilTag extends CommandBase {
    private final ScrappyAutoBase m_base;
    private final Boolean m_isBackCamera;
    private Integer m_desiredId = null;
    private Pose2d m_offset = new Pose2d(0, 0, 0);
    public LocalizeWithAprilTag(ScrappyAutoBase base, boolean isBackCamera) {
        m_base = base;
        m_isBackCamera = isBackCamera;
    }

    public LocalizeWithAprilTag(ScrappyAutoBase base, boolean isBackCamera, Pose2d offset) {
        m_base = base;
        m_isBackCamera = isBackCamera;
        m_offset = offset;
    }

    public LocalizeWithAprilTag(ScrappyAutoBase base, boolean isBackCamera, int desiredId) {
        m_base = base;
        m_isBackCamera = isBackCamera;
        m_desiredId = desiredId;
    }

    @Override
    public void initialize() {
        double robotHeading = m_base.robot.m_drive.pose.heading.toDouble();

        double avgX = 0;
        double avgY = 0;
        int detections = 0;

        for (int i = 0; i < 3; i++) {
            List<AprilTagDetection> detectionList = m_base.getAprilTagDetections(m_isBackCamera);
            for (AprilTagDetection detection : detectionList) {
                if (m_desiredId == null || m_desiredId == detection.id) {
                    Pose2d estimatedPos = AprilTagLocalization.getRobotPositionFromTag(detection, robotHeading, m_isBackCamera);
                    avgX += estimatedPos.position.x;
                    avgY += estimatedPos.position.y;
                    detections++;
                }
                break;
            }
        }

        if (detections != 0) {
            avgX /= detections;
            avgY /= detections;
            m_base.robot.m_drive.pose = new Pose2d(avgX + m_offset.position.x, avgY + m_offset.position.y, robotHeading + + m_offset.heading.toDouble());
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
