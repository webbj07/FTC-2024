package org.firstinspires.ftc.team4100worlds.subsystem;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team4100worlds.util.MB1242;

public class Sensors extends SubsystemBase {
    public static final double OFFSET = 2.9;
    private final Rev2mDistanceSensor m_backLeftSensor, m_backRightSensor;
    private final MB1242 m_frontSensor, m_leftSensor, m_rightSensor;
    private final RevTouchSensor m_touchSensor;
    private final ElapsedTime m_timer = new ElapsedTime();

    public Sensors(final HardwareMap hwMap) {
        m_backLeftSensor = hwMap.get(Rev2mDistanceSensor.class, "DSL");
        m_backRightSensor = hwMap.get(Rev2mDistanceSensor.class, "DSR");

        m_frontSensor = hwMap.get(MB1242.class, "FrontDistance");
        m_leftSensor = hwMap.get(MB1242.class, "LeftDistance");
        m_rightSensor = hwMap.get(MB1242.class, "RightDistance");

        m_touchSensor = hwMap.get(RevTouchSensor.class, "sensor_touch");

        m_timer.reset();
    }

    public boolean isPressed() {
        return m_touchSensor.isPressed();
    }

    public double getBackLeftDistance() {
        return m_backLeftSensor.getDistance(DistanceUnit.INCH);
    }

    public double getBackRightDistance() {
        return m_backRightSensor.getDistance(DistanceUnit.INCH);
    }

    public double getFrontDistance() {
        return m_frontSensor.getDistance(DistanceUnit.INCH) + OFFSET;
    }

    public double getLeftDistance() {
        return m_leftSensor.getDistance(DistanceUnit.INCH) + OFFSET;
    }

    public double getRightDistance() {
        return m_rightSensor.getDistance(DistanceUnit.INCH) + OFFSET;
    }

    public double getFrontDistanceAsync() {
        m_frontSensor.ping();
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 100) {

        }
        return getFrontDistance();
    }

    public double getLeftDistanceAsync() {
        m_leftSensor.ping();
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 100) {

        }
        return getLeftDistance();
    }

    public double getRightDistanceAsync() {
        m_rightSensor.ping();
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 100) {

        }
        return getRightDistance();
    }

    public void pingAll() {
        m_frontSensor.ping();
        m_leftSensor.ping();
        m_rightSensor.ping();
    }

    enum Sensor {
        LEFT,
        RIGHT,
        FRONT
    }
}
