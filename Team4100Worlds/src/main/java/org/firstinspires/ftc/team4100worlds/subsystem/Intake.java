package org.firstinspires.ftc.team4100worlds.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {

    public static final double GRAB_POS = 1.1;
    public static final double RETRACT_POS = 0.69;
    public static final double DOWN_POS = 0.71;
    public static final double UP_POS = 0.05;
    private final Servo m_grabber;
    private final Servo m_swivel;

    private boolean grabbed = false;
    private boolean rotatedOut = false;

    public Intake(final HardwareMap hwMap) {
        m_grabber = hwMap.get(Servo.class, "claw");
        m_swivel = hwMap.get(Servo.class, "tilt");
    }

    public void getGrabberPosition() {
        m_grabber.getPosition();
    }

    public void grab() {
        m_grabber.setPosition(GRAB_POS);
    }

    public void retract() {
        m_grabber.setPosition(RETRACT_POS);
    }
    public void toggleGrabber() {
        if (grabbed){
            retract();
            grabbed = false;
        }else{
            grab();
            grabbed = true;
        }
    }
    public void toggleRotation(){
        if (rotatedOut) {
            m_swivel.setPosition(0.05);
            rotatedOut = !rotatedOut;
        }else{
            m_swivel.setPosition(0.71);
            rotatedOut = !rotatedOut;

        }
    }
    public double getRotation(){return m_swivel.getPosition();}

}
