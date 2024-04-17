package org.firstinspires.ftc.team4100worlds.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team4100worlds.subsystem.Sensors;

public class WaitForPartner extends CommandBase {
    public static final double WAIT_THRESHOLD = 25;
    private final Sensors m_sensors;
    private final boolean m_isLeft;
    private boolean m_isFinished = false;
    private ElapsedTime m_autoTime = null;

    public WaitForPartner(Sensors sensors, boolean isLeft) {
        m_sensors = sensors;
        m_isLeft = isLeft;
    }

    public WaitForPartner(Sensors sensors, boolean isLeft, ElapsedTime autoTime) {
        m_sensors = sensors;
        m_isLeft = isLeft;
        m_autoTime = autoTime;
    }

    @Override
    public void execute() {
//        if (m_autoTime != null && m_autoTime.seconds() >= WAIT_THRESHOLD) {
//            m_isFinished = true;
//            return;
//        }

        if (m_isLeft) {
            if (m_sensors.getLeftDistanceAsync() > 37) {
                m_isFinished = true;
            }
        } else {
            if (m_sensors.getRightDistanceAsync() > 37) {
                m_isFinished = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
