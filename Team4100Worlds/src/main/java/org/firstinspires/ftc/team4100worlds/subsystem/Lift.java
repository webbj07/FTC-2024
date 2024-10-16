package org.firstinspires.ftc.team4100worlds.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends SubsystemBase {
    public static int LIFT_TOLERANCE = 50;
    private final DcMotorEx m_lift;

    public Lift(final HardwareMap hwMap) {
        m_lift = hwMap.get(DcMotorEx.class, "");
        m_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        m_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void run() {m_lift.setPower(100);}
    public void stop() {m_lift.setPower(0);}

    public void setPosition(int pos) {
        m_lift.setTargetPosition(pos);
    }

    public void setPower(double power) {
        m_lift.setPower(power);
    }

    public boolean isWithinTolerance(double target) {
        return Math.abs(target - m_lift.getCurrentPosition()) <= LIFT_TOLERANCE;
    }
}