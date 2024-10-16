package org.firstinspires.ftc.team4100worlds.subsystem;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    public static PIDCoefficients PID = new PIDCoefficients(0.013, 0, 0);
    private static final double GRAB_POS = 0.3, RELEASE_POS = 0;
    public static int LIFT_TOLERANCE = 50;
    private final DcMotor m_arm;
    private final Servo armGrabber;
    public Arm(final HardwareMap hwMap) {
        m_arm = hwMap.get(DcMotor.class, "Lift");
//        m_arm.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PID);
//        m_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m_arm.setDirection(DcMotorSimple.Direction.REVERSE);
//        m_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        m_arm.setTargetPosition(0);
//        m_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        m_arm.setPower(0.8);
        armGrabber = hwMap.get(Servo.class, "topgrab");
    }

    public void resetPosition() {
        m_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getPosition() {
        return m_arm.getCurrentPosition();
    }

    public void setPower(double power) {
        m_arm.setPower(power);
    }

    public boolean isWithinTolerance(double target) {
        return Math.abs(target - getPosition()) <= LIFT_TOLERANCE;
    }
    public void runToPosition(int position) {
        m_arm.setTargetPosition(position);
    }
    public void grab() {
        armGrabber.setPosition(GRAB_POS);
    }
    public void release(){
        armGrabber.setPosition(RELEASE_POS);
    }
    public double getGrabberPosition() {
        return armGrabber.getPosition();
    }
    public void toggleGrab(){
        if (getGrabberPosition() == GRAB_POS){
            release();
        }else{
            grab();
        }
    }
}