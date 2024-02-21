package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalization;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class DriveToAprilTag extends CommandBase {
    private final ScrappyAutoBase m_base;
    private PoseVelocity2d m_offset;
    private Integer m_desiredId = null;
    private Boolean m_isBackCamera = null;
    private Action m_action = null;
    private boolean m_finished = false;

    public DriveToAprilTag(ScrappyAutoBase base, PoseVelocity2d offset) {
        m_base = base;
        m_offset = offset;
    }

    public DriveToAprilTag(ScrappyAutoBase base, int id, boolean isBackCamera, PoseVelocity2d offset) {
        m_base = base;
        m_desiredId = id;
        m_isBackCamera = isBackCamera;
        m_offset = offset;
    }

    @Override
    public void initialize() {
        m_isBackCamera = m_isBackCamera == null ? true : m_isBackCamera;

        double robotHeading = m_base.robot.m_drive.pose.heading.toDouble();

        double avgX = 0;
        double avgY = 0;
        int detections = 0;

        for (int i = 0; i < 3; i++) {
            List<AprilTagDetection> detectionList = m_base.getAprilTagDetections(m_isBackCamera);
            for (AprilTagDetection detection : detectionList) {
                if (m_desiredId == null || detection.id == m_desiredId) {
                    Pose2d estimatedPos = AprilTagLocalization.getRobotPositionFromTag(detection, robotHeading, m_isBackCamera);
                    avgX += estimatedPos.position.x;
                    avgY += estimatedPos.position.y;
                    detections++;
                    break;
                }
            }
        }

        if (detections != 0) {
            avgX /= detections;
            avgY /= detections;
            m_base.robot.m_drive.pose = new Pose2d(avgX, avgY, robotHeading);
        }

        m_action = m_base.robot.m_drive.actionBuilder(m_base.robot.m_drive.pose)
                .strafeToLinearHeading(AprilTagLocalization.getTagPosition(m_desiredId != null ? m_desiredId : 1).plus(m_offset.component1()), Math.PI + m_offset.component2())
                .build();
    }

    @Override
    public void execute() {
        if (m_action != null) {
            TelemetryPacket packet = new TelemetryPacket();
            m_action.preview(packet.fieldOverlay());
            m_finished = !m_action.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        } else {
            m_finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
