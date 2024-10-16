package org.firstinspires.ftc.team4100worlds;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team4100worlds.pedropathing.follower.Follower;
import org.firstinspires.ftc.team4100worlds.subsystem.Arm;
import org.firstinspires.ftc.team4100worlds.subsystem.Extendo;
import org.firstinspires.ftc.team4100worlds.subsystem.Intake;

public class ScrappyCore extends Robot {
    public final ScrappyConstants.AllianceType ALLIANCE_TYPE;
    public final ScrappyConstants.AllianceSide ALLIANCE_SIDE;

    // Subsystems
    public Follower m_drive;
    public final Arm arm;
    public final Extendo extendo;
    public final Intake intake;
    //public final Lift lift;
    public ScrappyCore(HardwareMap hardwareMap, ScrappyConstants.AllianceType allianceType, ScrappyConstants.AllianceSide allianceSide) {
        ALLIANCE_TYPE = allianceType;
        ALLIANCE_SIDE = allianceSide;

        // Schedule to clear cache continuously (manual mode)

        // Initialize subsystems
        m_drive = new Follower(hardwareMap);
        arm = new Arm(hardwareMap);
        extendo = new Extendo(hardwareMap);
        intake = new Intake(hardwareMap);
        //lift = new Lift(hardwareMap);
    }
}