package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake extends SubsystemBase {
    public static double GRABBER_ONE_IDLE_POS = 0;
    public static double GRABBER_TWO_IDLE_POS = 0;
    public static double GRABBER_ONE_GRAB_POS = 1;
    public static double GRABBER_TWO_GRAB_POS = 1;

    private final DcMotorEx m_intake;
    private final Servo m_grabberOne;
    private final Servo m_grabberTwo;
    private double m_intakeSpeed = 1;

    public Intake(final HardwareMap hwMap) {
        m_intake = hwMap.get(DcMotorEx.class, "Intake");
        m_grabberOne = hwMap.get(Servo.class, "Grabber1");
        m_grabberTwo = hwMap.get(Servo.class, "Grabber2");

        m_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_grabberOne.setPosition(GRABBER_ONE_IDLE_POS);
        m_grabberTwo.setPosition(GRABBER_TWO_IDLE_POS);

        this.stop();
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
