package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane extends SubsystemBase {
    public final static double LAUNCH_POS = 0.95;
    public final static double BACK_POS = 0.8;
    private final Servo m_plane;

    public Plane(final HardwareMap hwMap) {
        m_plane = hwMap.get(Servo.class, "Plane");
    }

    public void launch() {
        m_plane.setPosition(LAUNCH_POS);
    }

    public void launch(double pos) {
        m_plane.setPosition(LAUNCH_POS + pos);
    }

    public void back() {
        m_plane.setPosition(BACK_POS);
    }

    public void back(double pos) {
        m_plane.setPosition(BACK_POS + pos);
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
