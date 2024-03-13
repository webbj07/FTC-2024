package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class Lift extends SubsystemBase {
    public static double MAX_VEL = 2190;
    public static double MAX_ACCEL = 2000;
    public static PIDCoefficients PID = new PIDCoefficients(0, 0 , 0);
    public static double kV = 0.00051;
    public static double kA = 0.0001;
    public static double kG = 0;
    public static int LIFT_TOLERANCE = 50;
    private final DcMotorEx m_leftMotor, m_rightMotor;

    public Lift(final HardwareMap hwMap) {
        m_leftMotor = hwMap.get(DcMotorEx.class, "LSlide");
        m_rightMotor = hwMap.get(DcMotorEx.class, "RSlide");

        m_leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        m_leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        m_leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getPosition() {
        return (m_leftMotor.getCurrentPosition() + m_rightMotor.getCurrentPosition()) / 2;
    }

    public double getVelocity() {
        return (m_leftMotor.getVelocity() + m_rightMotor.getVelocity()) / 2;
    }

    public void setPower(double power) {
        m_leftMotor.setPower(power);
        m_rightMotor.setPower(power);
    }

    public boolean isWithinTolerance(double target) {
        return Math.abs(target - m_leftMotor.getCurrentPosition()) <= LIFT_TOLERANCE;
    }
}