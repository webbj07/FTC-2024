package org.firstinspires.ftc.team4100worlds.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team4100worlds.subsystem.Sensors;

public class WaitForPartner extends CommandBase {
    private final Sensors m_sensors;
    private final boolean m_isLeft;
    private ElapsedTime m_timer = new ElapsedTime();
    private boolean m_isFinished = false;

    public WaitForPartner(Sensors sensors, boolean isLeft) {
        m_sensors = sensors;
        m_isLeft = isLeft;
        m_timer.reset();
    }

    @Override
    public void execute() {
        m_sensors.pingAll();
        while (m_timer.milliseconds() < 100) {}

        if (m_isLeft) {
            if (m_sensors.getFrontDistance() > 20) {
                m_isFinished = true;
            }
        } else {
            if (m_sensors.getFrontDistance() > 21) {
                m_isFinished = true;
            }
        }

        m_timer.reset();
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
