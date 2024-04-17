package org.firstinspires.ftc.team4100worlds.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100worlds.pedropathing.follower.Follower;

public class WaitForReachedTValue extends CommandBase {
    private final Follower m_follower;
    private final double m_tValue;
    private Integer m_pathNumber = null;
    private boolean m_isFinished = false;

    public WaitForReachedTValue(Follower follower, double tValue) {
        m_follower = follower;
        m_tValue = tValue;
    }

    public WaitForReachedTValue(Follower follower, int pathNumber, double tValue) {
        m_follower = follower;
        m_pathNumber = pathNumber;
        m_tValue = tValue;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_pathNumber != null && m_follower.getCurrentPathNumber() == m_pathNumber) {
            if (m_follower.getCurrentTValue() >= m_tValue) {
                m_isFinished = true;
            }
        } else if (m_pathNumber == null && m_follower.getCurrentTValue() >= m_tValue) {
            m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
