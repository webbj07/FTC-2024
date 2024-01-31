package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Conveyor extends SubsystemBase {
    private final DcMotorEx m_conveyor;
    private double m_conveyorSpeed = 1;

    public Conveyor(final HardwareMap hwMap) {
        m_conveyor = hwMap.get(DcMotorEx.class, "Conveyor");
        m_conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_conveyor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void up() {
        m_conveyor.setPower(m_conveyorSpeed);
    }

    public void down() {
        m_conveyor.setPower(-m_conveyorSpeed);
    }

    public void stop() {
        m_conveyor.setPower(0);
    }

    public void setSpeed(double speed) {
        m_conveyorSpeed = Range.clip(speed, 0, 1);
    }

    public double getSpeed() {
        return m_conveyorSpeed;
    }
}
