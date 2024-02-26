package org.firstinspires.ftc.team4100worlds.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends SubsystemBase {
    public final static double DROP_POS = 0.82;
    public final static double BACK_POS = 0.68;
    public final static double EXTEND_LOW = 0.96;
    public final static double EXTEND_HIGH = 0.51;
    private final Servo m_dropper, m_leftExtend, m_rightExtend;

    public Outtake(final HardwareMap hwMap) {
        m_dropper = hwMap.get(Servo.class, "Dropper");
        m_leftExtend = hwMap.get(Servo.class, "ExtendL");
        m_rightExtend = hwMap.get(Servo.class, "ExtendR");

//        m_rightExtend.setDirection(Servo.Direction.REVERSE);

        m_leftExtend.setPosition(EXTEND_LOW);
//        m_rightExtend.setPosition(EXTEND_LOW);
    }

    public void extend() {
        m_leftExtend.setPosition(EXTEND_HIGH);
//        m_rightExtend.setPosition(EXTEND_HIGH);
    }

    public void extend(double pos) {
        m_leftExtend.setPosition(EXTEND_HIGH + pos + 0.14);
//        m_rightExtend.setPosition(EXTEND_HIGH + pos);
    }

    public void lower() {
        m_leftExtend.setPosition(EXTEND_LOW);
//        m_rightExtend.setPosition(EXTEND_LOW);
    }

    public void lower(double pos) {
        m_leftExtend.setPosition(EXTEND_LOW + pos);
//        m_rightExtend.setPosition(EXTEND_LOW);
    }

    public void setRelExtendPos(double pos) {
        m_leftExtend.setPosition(pos + m_leftExtend.getPosition());
//        m_rightExtend.setPosition(pos + m_rightExtend.getPosition());
    }

    public void setExtendPos(double pos) {
        m_leftExtend.setPosition(pos);
//        m_rightExtend.setPosition(pos);
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

    public void setDropperPos(double pos) {
        m_dropper.setPosition(pos);
    }

    public void setRelDropperPos(double pos) {
        m_dropper.setPosition(m_dropper.getPosition() + pos);
    }

    public double getDropperPos() {
        return m_dropper.getPosition();
    }
}
