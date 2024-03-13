package org.firstinspires.ftc.team4100worlds.subsystem;

import static org.firstinspires.ftc.team4100worlds.ScrappyConstants.Positions.Plane.LAUNCH;
import static org.firstinspires.ftc.team4100worlds.ScrappyConstants.Positions.Plane.IDLE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane extends SubsystemBase {
    private final Servo m_plane;

    public Plane(final HardwareMap hwMap) {
        m_plane = hwMap.get(Servo.class, "Plane");
    }

    public void launch() {
        m_plane.setPosition(LAUNCH);
    }

    public void launch(double pos) {
        m_plane.setPosition(LAUNCH + pos);
    }

    public void back() {
        m_plane.setPosition(IDLE);
    }

    public void back(double pos) {
        m_plane.setPosition(IDLE + pos);
    }

    public void setPos(double pos) {
        m_plane.setPosition(pos);
    }

    public void setRelPos(double pos) {
        m_plane.setPosition(m_plane.getPosition() + pos);
    }

    public double getPos() {
        return m_plane.getPosition();
    }
}
