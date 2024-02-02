package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorLocalization extends SubsystemBase {
    private final DistanceSensor m_leftSensor, m_rightSensor;
    public SensorLocalization(final HardwareMap hwMap) {
        m_leftSensor = hwMap.get(DistanceSensor.class, "LS");
        m_rightSensor = hwMap.get(DistanceSensor.class, "RS");
    }

    public double getBackboardHeadingError() {
        double d1 = m_leftSensor.getDistance(DistanceUnit.INCH);
        double d2 = m_rightSensor.getDistance(DistanceUnit.INCH);

        return Math.atan(Math.abs(d1 - d2) / 15);
    }
}
