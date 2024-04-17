package org.firstinspires.ftc.team4100worlds.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team4100worlds.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;

public class GotoStack extends CommandBase {
    private final ScrappyAutoBase m_base;
    private final ElapsedTime m_elapsedTime = new ElapsedTime();

    public GotoStack(ScrappyAutoBase base) {
        m_base = base;
    }

    @Override
    public void initialize() {
        Path stackPath = new Path(new BezierLine(new Point(m_base.robot.m_drive.getPose()), new Point(new Pose2d(-55.5, m_base.robot.m_drive.poseUpdater.getPose().getY(), Math.PI))));
        stackPath.setConstantHeadingInterpolation(Math.PI);
        stackPath.setPathEndTValue(0.9);
        stackPath.setPathEndTimeout(1);

        m_base.robot.m_drive.setMaxPower(0.35);
        m_base.robot.m_drive.followPath(stackPath);
        m_elapsedTime.reset();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (!m_base.robot.m_drive.isBusy() || m_elapsedTime.seconds() > 2.1) {
            m_base.robot.m_drive.setMaxPower(1);
            m_base.robot.m_drive.poseUpdater.resetOffset();
            return true;
        }

        return false;
    }
}
