package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalization;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class DriveToAprilTag extends CommandBase {
    private final ScrappyAutoBase m_base;
    private final int m_desiredId;
    private final boolean m_isBackCamera;
    private final double m_disAway;
    private Action m_action = null;
    private boolean m_finished = false;
    public DriveToAprilTag(ScrappyAutoBase base, int id, boolean isBackCamera, double disAway) {
        m_base = base;
        m_desiredId = id;
        m_isBackCamera = isBackCamera;
        m_disAway = disAway;
    }

    @Override
    public void initialize() {
        double robotHeading = m_base.robot.m_drive.pose.heading.toDouble();
        double avgX = 0;
        double avgY = 0;

        for (int i = 0; i < 3; i++) {
            for (AprilTagDetection detection : m_base.getAprilTagDetections(m_isBackCamera)) {
                if (detection.id == m_desiredId && detection.metadata != null) {
                    Pose2d estimatedPos = AprilTagLocalization.getRobotPositionFromTag(detection, robotHeading, m_isBackCamera);
                    avgX += estimatedPos.position.x;
                    avgY += estimatedPos.position.y;
                    break;
                }
            }
        }

        if (avgX == 0 && avgY == 0) {
            m_finished = true;
            return;
        }

        avgX /= 3;
        avgY /= 3;

        m_base.robot.m_drive.pose = new Pose2d(avgX, avgY, robotHeading);
        m_action = m_base.robot.m_drive.actionBuilder(m_base.robot.m_drive.pose)
                .strafeToLinearHeading(AprilTagLocalization.getTagPosition(m_desiredId).plus(new Vector2d(m_desiredId <= 6 ? -m_disAway : m_disAway, 0)), robotHeading)
                .build();
    }

    @Override
    public void execute() {
        if (m_action != null) {
            TelemetryPacket packet = new TelemetryPacket();
            m_action.preview(packet.fieldOverlay());
            m_finished = !m_action.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
