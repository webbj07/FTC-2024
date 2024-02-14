package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.ScrappySettings.DISTANCE_SENSOR_WIDTH;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorLocalization extends SubsystemBase {
    public final DistanceSensor m_leftSensor, m_rightSensor;
    public SensorLocalization(final HardwareMap hwMap) {
        m_leftSensor = hwMap.get(DistanceSensor.class, "DSL");
        m_rightSensor = hwMap.get(DistanceSensor.class, "DSR");
    }

    public double getLeftDistance() {
        return m_leftSensor.getDistance(DistanceUnit.INCH);
    }

    public double getRightDistance() {
        return m_rightSensor.getDistance(DistanceUnit.INCH);
    }

    public double getBackboardHeadingError() {
        double d1 = m_leftSensor.getDistance(DistanceUnit.INCH);
        double d2 = m_rightSensor.getDistance(DistanceUnit.INCH);
        double factor = 1;

        if (d1 > d2) {
            factor = -1;
        }

        double error = factor * Math.atan2(Math.abs(d1 - d2), DISTANCE_SENSOR_WIDTH);

        // safety
        if (Math.abs(error) > Math.toRadians(30)) {
            return 0;
        }

        return error;
    }

    public double getTurnToHeading(double currentHeading) {
        return currentHeading + getBackboardHeadingError();
    }
}
