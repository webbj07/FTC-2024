package org.firstinspires.ftc.team4100worlds.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100worlds.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;

import java.util.ArrayList;

public class LocalizeWithStack extends CommandBase {
    private final ScrappyAutoBase m_base;
    private boolean m_finished = false;

    public LocalizeWithStack(ScrappyAutoBase base) {
        m_base = base;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_base.robot.m_drive.getPose();
        double robotHeading = currentPose.getHeading();

        ArrayList<Vector2d> errorList = new ArrayList<>();

        while (errorList.size() < 3) {
            Vector2d error = m_base.stackProcessor.getDistanceError(robotHeading);
            if (error != null) {
                errorList.add(error);
            }
        }

        double errorX = 0;
        double errorY = 0;

        for (Vector2d err : errorList) {
            errorX += err.getX();
            errorY += err.getY();
        }

        errorX /= 3;
        errorY /= 3;

        Path stackPath = new Path(new BezierLine(new Point(currentPose), new Point(new Pose2d(currentPose.getX() + errorX + 12.5, currentPose.getY() + errorY + 1.5, Math.PI))));
        stackPath.setConstantHeadingInterpolation(Math.PI);
        stackPath.setPathEndTValue(0.9);
        stackPath.setPathEndTimeout(1);

        m_base.robot.m_drive.followPath(stackPath);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return !m_base.robot.m_drive.isBusy();
    }
}
