package org.firstinspires.ftc.team4100worlds.subsystem;

import static org.firstinspires.ftc.team4100worlds.ScrappyConstants.Positions.Outtake.EXTENSION_MIN;
import static org.firstinspires.ftc.team4100worlds.ScrappyConstants.Positions.Outtake.EXTENSION_MAX;
import static org.firstinspires.ftc.team4100worlds.ScrappyConstants.Positions.Outtake.DROPPER_IN;
import static org.firstinspires.ftc.team4100worlds.ScrappyConstants.Positions.Outtake.DROPPER_OUT;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends SubsystemBase {
    private final Servo m_dropper, m_leftExtend;

    public Outtake(final HardwareMap hwMap) {
        m_dropper = hwMap.get(Servo.class, "Dropper");
        m_leftExtend = hwMap.get(Servo.class, "ExtendL");

        m_leftExtend.setPosition(EXTENSION_MIN);
    }

    public double getExPos() {
        return m_leftExtend.getPosition();
    }

    public void extend() {
        m_leftExtend.setPosition(EXTENSION_MAX);
    }

    public void extend(double pos) {
        m_leftExtend.setPosition(EXTENSION_MAX + pos);
    }

    public void lower() {
        m_leftExtend.setPosition(EXTENSION_MIN);
    }

    public void lower(double pos) {
        m_leftExtend.setPosition(EXTENSION_MIN + pos);
    }

    public void setRelExtendPos(double pos) {
        m_leftExtend.setPosition(pos + m_leftExtend.getPosition());
    }

    public void setExtendPos(double pos) {
        m_leftExtend.setPosition(pos);
    }

    public void drop() {
        m_dropper.setPosition(DROPPER_OUT);
    }

    public void drop(double pos) {
        m_dropper.setPosition(DROPPER_OUT + pos);
    }

    public void back() {
        m_dropper.setPosition(DROPPER_IN);
    }

    public void back(double pos) {
        m_dropper.setPosition(DROPPER_IN + pos);
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
