package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalization;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class LocalizeWithStack extends CommandBase {
    private final ScrappyAutoBase m_base;
    private Action m_action = null;
    private boolean m_finished = false;

    public LocalizeWithStack(ScrappyAutoBase base) {
        m_base = base;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_base.robot.m_drive.pose;
        double robotHeading = currentPose.heading.toDouble();

        Vector2d error = m_base.stackProcessor.getDistanceError(robotHeading);
        while (error == null) {
            error = m_base.stackProcessor.getDistanceError(robotHeading);
        }

        m_action = m_base.robot.m_drive.actionBuilder(m_base.robot.m_drive.pose)
                .strafeToLinearHeading(new Vector2d(currentPose.position.x + error.x + 10, currentPose.position.y + error.y + 1.5), Math.PI)
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
