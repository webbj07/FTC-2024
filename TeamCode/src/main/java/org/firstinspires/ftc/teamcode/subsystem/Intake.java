package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake extends SubsystemBase {
    public static double EXTEND_UP_POS = 0.13;
    public static double EXTEND_DOWN_POS = 0.49;
    public static double GRABBER_ONE_IDLE_POS = 0.3;
    public static double GRABBER_TWO_IDLE_POS = 0.48;
    public static double GRABBER_ONE_GRAB_POS = 0.6;
    public static double GRABBER_TWO_GRAB_POS = 0.13;
    private final DcMotorEx m_intake;
    public final Servo m_intakeEx, m_grabberOne, m_grabberTwo;
    private double m_intakeSpeed = 1;

    public Intake(final HardwareMap hwMap) {
        m_intake = hwMap.get(DcMotorEx.class, "Intake");
        m_intakeEx = hwMap.get(Servo.class, "IntakeEx");
        m_grabberOne = hwMap.get(Servo.class, "Grabber1");
        m_grabberTwo = hwMap.get(Servo.class, "Grabber2");

        m_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_grabberOne.setPosition(GRABBER_ONE_IDLE_POS);
        m_grabberTwo.setPosition(GRABBER_TWO_IDLE_POS);

        this.stop();
    }

    public void setGrabOnePos(double pos) {
        m_grabberOne.setPosition(m_grabberOne.getPosition() + pos);
    }

    public void setGrabTwoPos(double pos) {
        m_grabberTwo.setPosition(m_grabberTwo.getPosition() + pos);
    }

    public void exUp() {
        m_intakeEx.setPosition(EXTEND_UP_POS);
    }

    public void exDown() {
        m_intakeEx.setPosition(EXTEND_DOWN_POS);
    }

    public void grab() {
        m_grabberOne.setPosition(GRABBER_ONE_GRAB_POS);
        m_grabberTwo.setPosition(GRABBER_TWO_GRAB_POS);
    }

    public void grab(double rel) {
        m_grabberOne.setPosition(GRABBER_ONE_GRAB_POS + rel);
        m_grabberTwo.setPosition(GRABBER_TWO_GRAB_POS + rel);
    }

    public void back() {
        m_grabberOne.setPosition(GRABBER_ONE_IDLE_POS);
        m_grabberTwo.setPosition(GRABBER_TWO_IDLE_POS);
    }

    public void backOne() {
        m_grabberOne.setPosition(GRABBER_ONE_IDLE_POS);
    }

    public void backTwo() {
        m_grabberTwo.setPosition(GRABBER_TWO_IDLE_POS);
    }

    public void back(double rel) {
        m_grabberOne.setPosition(GRABBER_ONE_IDLE_POS + rel);
        m_grabberTwo.setPosition(GRABBER_TWO_IDLE_POS + rel);
    }

    public void suck() {
        m_intake.setPower(m_intakeSpeed);
    }

    public void spit() {
        m_intake.setPower(-m_intakeSpeed);
    }

    public void stop() {
        m_intake.setPower(0);
    }

    public void setSpeed(double speed) {
        m_intakeSpeed = Range.clip(speed, 0, 1);
    }

    public double getSpeed() {
        return m_intakeSpeed;
    }

    public double getPower() {
        return m_intake.getPower();
    }

    public double getCurrent() {
        return m_intake.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isSucking() {
        return m_intake.getPower() == m_intakeSpeed;
    }

    public boolean isSpitting() {
        return m_intake.getPower() == -m_intakeSpeed;
    }
}
