package org.firstinspires.ftc.team4100worlds.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100worlds.pedropathing.follower.Follower;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;

public class FollowPath extends CommandBase {
    private final Follower m_follower;
    private Path m_path = null;
    private PathChain m_pathChain = null;
    private double m_power = 1.0;

    public FollowPath(Follower follower, Path path) {
        m_follower = follower;
        m_path = path;
    }

    public FollowPath(Follower follower, PathChain path) {
        m_follower = follower;
        m_pathChain = path;
    }

    public FollowPath(Follower follower, PathChain path, double power) {
        m_follower = follower;
        m_pathChain = path;
        m_power = power;
    }

    @Override
    public void initialize() {
        m_follower.setMaxPower(m_power);

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
        if (!m_follower.isBusy()) {
            m_follower.setMaxPower(1);
            return true;
        }
        return false;
    }
}
