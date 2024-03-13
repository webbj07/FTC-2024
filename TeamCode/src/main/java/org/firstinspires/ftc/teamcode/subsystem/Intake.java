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
    public static double EXTEND_DOWN_POS = 0.44;
    public static double GRABBER_ONE_IDLE_POS = 0.3;
    public static double GRABBER_TWO_IDLE_POS = 0.48;
    public static double GRABBER_ONE_GRAB_POS = 0.6;
    public static double GRABBER_TWO_GRAB_POS = 0.13;
    private final DcMotorEx m_intake;
    private final Servo m_intakeEx, m_grabberOne, m_grabberTwo;
    private double m_intakeSpeed = 1;

    public Intake(final HardwareMap hwMap) {
        m_intake = hwMap.get(DcMotorEx.class, "Intake");
        m_intakeEx = hwMap.get(Servo.class, "IntakeEx");
        m_grabberOne = hwMap.get(Servo.class, "Grabber1");
        m_grabberTwo = hwMap.get(Servo.class, "Grabber2");

        m_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.raise();
        this.grab();

        this.stop();
    }

    public void setExtendRelPos(double pos) {
        m_intakeEx.setPosition(m_intakeEx.getPosition() + pos);
    }

    public double getExtendPos() {
        return m_intakeEx.getPosition();
    }

    public void raise() {
        m_intakeEx.setPosition(EXTEND_UP_POS);
    }

    public void lower() {
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

    public void back(double rel) {
        m_grabberOne.setPosition(GRABBER_ONE_IDLE_POS + rel);
        m_grabberTwo.setPosition(GRABBER_TWO_IDLE_POS + rel);
    }

    public void backOne() {
        m_grabberOne.setPosition(GRABBER_ONE_IDLE_POS);
    }

    public void backTwo() {
        m_grabberTwo.setPosition(GRABBER_TWO_IDLE_POS);
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

    public void setIntakeSpeed(double speed) {
        m_intakeSpeed = Range.clip(speed, 0, 1);
    }

    public double getIntakeSpeed() {
        return m_intakeSpeed;
    }

    public double getIntakePower() {
        return m_intake.getPower();
    }

    public double getIntakeCurrent() {
        return m_intake.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isSucking() {
        return m_intake.getPower() > 0;
    }

    public boolean isSpitting() {
        return m_intake.getPower() < 0;
    }
}
