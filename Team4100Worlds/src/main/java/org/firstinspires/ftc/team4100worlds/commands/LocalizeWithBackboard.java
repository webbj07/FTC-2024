package org.firstinspires.ftc.team4100worlds.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team4100worlds.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierPoint;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;

public class LocalizeWithBackboard extends CommandBase {
    public static double MAX_ATTEMPTS = 10;
    private final ScrappyAutoBase m_base;
    private final double m_dist;
    private final ElapsedTime m_timer = new ElapsedTime();
    private Pose2d m_offset = new Pose2d();
    private boolean m_saveOffset = false;
    private boolean m_holdCurrentPoint = false;
    private int m_attempts = 0;
    private boolean m_finished = false;

    public LocalizeWithBackboard(ScrappyAutoBase base, double dist) {
        m_base = base;
        m_dist = dist;
    }

    public LocalizeWithBackboard(ScrappyAutoBase base, double dist, boolean saveOffset) {
        m_base = base;
        m_dist = dist;
        m_saveOffset = saveOffset;
    }

    public LocalizeWithBackboard(ScrappyAutoBase base, double dist, boolean saveOffset, boolean holdCurrentPoint) {
        m_base = base;
        m_dist = dist;
        m_saveOffset = saveOffset;
        m_holdCurrentPoint = holdCurrentPoint;
    }

    @Override
    public void initialize() {
        m_base.robot.m_drive.setMaxPower(0.4);
        m_base.robot.m_drive.holdPoint(new BezierPoint(new Point(m_base.robot.m_drive.getPose())), Math.PI);
        m_timer.reset();
    }

    @Override
    public void execute() {
        if (m_attempts++ > MAX_ATTEMPTS) {
            m_finished = true;
            return;
        }

        if (m_timer.milliseconds() > 20) {
            double dL = m_base.robot.m_sensors.getBackLeftDistance();
            double dR = m_base.robot.m_sensors.getBackRightDistance();

            double realDist = (dL + dR) / 2;

            if (m_saveOffset) {
                m_offset = m_base.robot.m_drive.poseUpdater.getOffset();
                m_base.robot.m_drive.poseUpdater.setRelXOffset(m_dist - realDist);
            } else {
                m_base.robot.m_drive.poseUpdater.setXOffset(m_dist - realDist);
            }

            m_timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        if (m_finished) {
            m_base.robot.m_drive.breakFollowing();

            if (m_saveOffset) {
                m_base.robot.m_drive.poseUpdater.setXOffset(m_offset.getX());
                m_base.robot.m_drive.poseUpdater.setYOffset(m_offset.getY());
                m_base.robot.m_drive.poseUpdater.setHeadingOffset(m_offset.getHeading());
            } else {
                m_base.robot.m_drive.poseUpdater.resetOffset();
            }

            m_base.robot.m_drive.setMaxPower(1);
            return true;
        }
        return false;
    }
}
