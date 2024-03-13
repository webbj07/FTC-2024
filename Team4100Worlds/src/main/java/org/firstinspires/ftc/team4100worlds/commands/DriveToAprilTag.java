package org.firstinspires.ftc.team4100worlds.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100worlds.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.team4100worlds.pedropathing.follower.Follower;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;
import org.firstinspires.ftc.team4100worlds.vision.AprilTagLocalization;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class DriveToAprilTag extends CommandBase {
    private final ScrappyAutoBase m_base;
    private final Pose2d m_offset;
    private Integer m_desiredId = null;
    private Boolean m_isBackCamera = null;

    public DriveToAprilTag(ScrappyAutoBase base, Pose2d offset) {
        m_base = base;
        m_offset = offset;
    }

    public DriveToAprilTag(ScrappyAutoBase base, int id, boolean isBackCamera, Pose2d offset) {
        m_base = base;
        m_desiredId = id;
        m_isBackCamera = isBackCamera;
        m_offset = offset;
    }

    @Override
    public void initialize() {
        m_isBackCamera = m_isBackCamera == null || m_isBackCamera;

        double robotHeading = m_base.robot.m_drive.getPose().getHeading();

        double avgX = 0;
        double avgY = 0;
        int detections = 0;

        for (int i = 0; i < 3; i++) {
            List<AprilTagDetection> detectionList = m_base.getAprilTagDetections(m_isBackCamera);
            for (AprilTagDetection detection : detectionList) {
                if (m_desiredId == null || detection.id == m_desiredId) {
                    Pose2d estimatedPos = AprilTagLocalization.getRobotPositionFromTag(detection, robotHeading, m_isBackCamera);
                    avgX += estimatedPos.getX();
                    avgY += estimatedPos.getY();
                    detections++;
                    break;
                }
            }
        }

        if (detections != 0) {
            avgX /= detections;
            avgY /= detections;
            m_base.robot.m_drive.poseUpdater.setCurrentPoseUsingOffset(new Pose2d(avgX, avgY, robotHeading));
        }

        Vector2d tagPos = AprilTagLocalization.getTagPosition(m_desiredId != null ? m_desiredId : 1);

        Path path = new Path(new BezierLine(
                new Point(avgX, avgY, Point.CARTESIAN),
                new Point(tagPos.getX() + m_offset.getX(), tagPos.getY() + m_offset.getY(), Point.CARTESIAN)
        ));
        path.setConstantHeadingInterpolation(Math.PI);

        m_base.robot.m_drive.setMaxPower(0.5);
        m_base.robot.m_drive.followPath(path);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (!m_base.robot.m_drive.isBusy()) {
            m_base.robot.m_drive.setMaxPower(1);
            m_base.robot.m_drive.poseUpdater.resetOffset();
            return true;
        }

        return false;
    }
}
