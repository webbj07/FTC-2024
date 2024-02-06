package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalization;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class DriveToAprilTag extends CommandBase {
    private final ScrappyAutoBase m_base;
    private final int m_desiredId;
    private final boolean m_isBackCamera;
    private final double m_disAway;
    private Command m_runOnDetected = null;
    private Action m_action = null;
    private boolean m_finished = false;
    public DriveToAprilTag(ScrappyAutoBase base, int id, boolean isBackCamera, double disAway) {
        m_base = base;
        m_desiredId = id;
        m_isBackCamera = isBackCamera;
        m_disAway = disAway;
    }

    public DriveToAprilTag(ScrappyAutoBase base, int id, boolean isBackCamera, double disAway, Command runOnDetected) {
        m_base = base;
        m_desiredId = id;
        m_isBackCamera = isBackCamera;
        m_disAway = disAway;
        m_runOnDetected = runOnDetected;
    }

    @Override
    public void initialize() {
        double robotHeading = m_base.robot.m_drive.pose.heading.toDouble();
        double avgX = 0;
        double avgY = 0;

        for (int i = 0; i < 5; i++) {
            List<AprilTagDetection> detectionList = m_base.getAprilTagDetections(m_isBackCamera);
            for (AprilTagDetection detection : detectionList) {
                if (detection.id == m_desiredId) {
                    Pose2d estimatedPos = AprilTagLocalization.getRobotPositionFromTag(detection, robotHeading, m_isBackCamera);
                    avgX = estimatedPos.position.x;
                    avgY = estimatedPos.position.y;
                    break;
                }
            }
        }

        if (avgX == 0 && avgY == 0) {
            m_finished = true;
            return;
        }

        if (m_runOnDetected != null) {
            m_runOnDetected.schedule();
        }

//        avgX /= 3;
//        avgY /= 3;

        m_base.robot.m_drive.pose = new Pose2d(avgX, avgY, robotHeading);
        m_action = m_base.robot.m_drive.actionBuilder(m_base.robot.m_drive.pose)
                .strafeToLinearHeading(AprilTagLocalization.getTagPosition(m_desiredId).plus(new Vector2d(m_desiredId <= 6 ? -m_disAway : m_disAway, 0)), Math.toRadians(180))
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
