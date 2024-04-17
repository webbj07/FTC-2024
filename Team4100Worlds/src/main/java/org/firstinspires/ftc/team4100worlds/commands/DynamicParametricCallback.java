package org.firstinspires.ftc.team4100worlds.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100worlds.pedropathing.follower.Follower;

public class DynamicParametricCallback extends CommandBase {
    private final Follower m_follower;
    private final Runnable m_toRun;
    private final double m_tValue;
    private boolean m_isFinished = false;

    public DynamicParametricCallback(Follower follower, Runnable toRun, double tValue) {
        m_follower = follower;
        m_toRun = toRun;
        m_tValue = tValue;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_follower.getCurrentTValue() >= m_tValue) {
            m_toRun.run();
            m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
