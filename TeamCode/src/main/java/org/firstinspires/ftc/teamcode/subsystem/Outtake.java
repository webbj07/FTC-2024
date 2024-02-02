package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends SubsystemBase {
    public final static double DROP_POS = 0.82;
    public final static double BACK_POS = 0.72;
    private final Servo m_dropper, m_leftExtend, m_rightExtend;

    public Outtake(final HardwareMap hwMap) {
        m_dropper = hwMap.get(Servo.class, "Dropper");
        m_leftExtend = hwMap.get(Servo.class, "ExtendL");
        m_rightExtend = hwMap.get(Servo.class, "ExtendR");
    }

    public void drop() {
        m_dropper.setPosition(DROP_POS);
    }

    public void drop(double pos) {
        m_dropper.setPosition(DROP_POS + pos);
    }

    public void back() {
        m_dropper.setPosition(BACK_POS);
    }

    public void back(double pos) {
        m_dropper.setPosition(BACK_POS + pos);
    }

    public void setPosition(double pos) {
        m_dropper.setPosition(pos);
    }

    public void setRelativePosition(double pos) {
        m_dropper.setPosition(m_dropper.getPosition() + pos);
    }

    public double getPosition() {
        return m_dropper.getPosition();
    }
}