package org.firstinspires.ftc.team4100worlds.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100worlds.pedropathing.follower.Follower;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierPoint;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;

public class FollowPath extends CommandBase {
    private final Follower m_follower;
    private Path m_path = null;
    private PathChain m_pathChain = null;

    public FollowPath(Follower follower, Path path) {
        m_follower = follower;
        m_path = path;
    }

    public FollowPath(Follower follower, PathChain path) {
        m_follower = follower;
        m_pathChain = path;
    }

    @Override
    public void initialize() {
        if (m_path != null) {
            m_follower.followPath(m_path);
        } else {
            m_follower.followPath(m_pathChain);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return !m_follower.isBusy();
    }
}