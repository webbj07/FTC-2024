package org.firstinspires.ftc.team4100worlds.subsystem;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team4100worlds.util.MB1242;

public class Sensors extends SubsystemBase {
    public static final double OFFSET = 2.3;
    private final MB1242 m_frontSensor;
    private final ElapsedTime m_timer = new ElapsedTime();

    public Sensors(final HardwareMap hwMap) {
        m_frontSensor = hwMap.get(MB1242.class, "FrontDistance");
        m_timer.reset();
    }

    public double getFrontDistance() {
        return m_frontSensor.getDistance(DistanceUnit.INCH) + OFFSET;
    }

    public double getFrontDistanceAsync() {
        m_frontSensor.ping();
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 100) {

        }
        return getFrontDistance();
    }

    public void pingAll() {
        m_frontSensor.ping();
    }

//    @Override
//    public void periodic() {
//        if (m_timer.milliseconds() >= 100) {
//            pingAll();
//            m_timer.reset();
//        }
//    }
}
