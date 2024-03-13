package org.firstinspires.ftc.team4100worlds.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

public class Lift extends SubsystemBase {
    public static double MAX_VEL = 2100;
    public static double MAX_ACCEL = 2000;
    public static PIDCoefficients PID = new PIDCoefficients(0.027, 0 , 0.0002);
    public static double kG = 0.2;
    public static int LIFT_TOLERANCE = 75;
    private final DcMotorEx m_leftMotor, m_rightMotor;

    public Lift(final HardwareMap hwMap) {
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

        m_leftMotor.setPower(1);
        m_rightMotor.setPower(1);

        m_leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        m_leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getPosition() {
        return m_rightMotor.getCurrentPosition();
    }

    public double getVelocity() {
        return m_rightMotor.getVelocity();
    }

    public void setPower(double power) {
        m_leftMotor.setPower(power);
        m_rightMotor.setPower(power);
    }

    public void gotoPos(int pos) {
        m_leftMotor.setTargetPosition(pos);
        m_rightMotor.setTargetPosition(pos);
    }

    public void gotoPosRel(int pos) {
        m_leftMotor.setTargetPosition(pos + m_leftMotor.getCurrentPosition());
        m_rightMotor.setTargetPosition(pos + m_rightMotor.getCurrentPosition());
    }

    public boolean isWithinTolerance(double target) {
        return Math.abs(target - getPosition()) <= LIFT_TOLERANCE;
    }
}