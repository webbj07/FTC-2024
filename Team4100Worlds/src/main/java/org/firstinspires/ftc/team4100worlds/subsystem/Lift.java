package org.firstinspires.ftc.team4100worlds.subsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends SubsystemBase {
    public static double MAX_VEL = 2100;
    public static double MAX_ACCEL = 2000;
    public static PIDCoefficients PID = new PIDCoefficients(0.01, 0, 0);
    public static double kG = 0.2;
    public static int LIFT_TOLERANCE = 75;
    //
    private final DcMotorEx m_leftMotor, m_rightMotor;
    private final PIDController m_pidController;
    private final double m_targetPosition = 0;

    public Lift(final HardwareMap hwMap) {
        m_pidController = new PIDController(PID.kP, PID.kI, PID.kD);
        m_leftMotor = hwMap.get(DcMotorEx.class, "LSlide");
        m_rightMotor = hwMap.get(DcMotorEx.class, "RSlide");

        m_leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        m_leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        m_leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        m_rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_leftMotor.setTargetPosition(0);
        m_rightMotor.setTargetPosition(0);

        m_leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m_leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_leftMotor.setPower(1);
        m_rightMotor.setPower(1);
    }

    public void reset() {
        m_leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getPosition() {
        return m_rightMotor.getCurrentPosition();
    }

    public void setPosition(int targetPosition) {
//        m_targetPosition = targetPosition;
        m_leftMotor.setTargetPosition(targetPosition);
        m_rightMotor.setTargetPosition(targetPosition);
    }

    public int getTargetPosition() {
        return m_rightMotor.getTargetPosition();
    }

    public void setPositionRel(int relPos) {
        m_leftMotor.setTargetPosition(m_leftMotor.getCurrentPosition() + relPos);
        m_rightMotor.setTargetPosition(m_rightMotor.getCurrentPosition() + relPos);
    }

    public double getVelocity() {
        return m_rightMotor.getVelocity();
    }

    public boolean isWithinTolerance(double target) {
        return Math.abs(target - getPosition()) <= LIFT_TOLERANCE;
    }

//    @Override
//    public void periodic() {
//        double power = m_pidController.calculate(getPosition(), this.m_targetPosition);
//
//        m_leftMotor.setPower(power);
//        m_rightMotor.setPower(power);
//    }
}