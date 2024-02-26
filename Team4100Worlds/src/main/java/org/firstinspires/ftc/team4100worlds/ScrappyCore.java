package org.firstinspires.ftc.team4100worlds;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team4100worlds.pedropathing.follower.Follower;
import org.firstinspires.ftc.team4100worlds.commands.BulkCacheHandler;
import org.firstinspires.ftc.team4100worlds.subsystem.Conveyor;
import org.firstinspires.ftc.team4100worlds.subsystem.Intake;
import org.firstinspires.ftc.team4100worlds.subsystem.Lift;
import org.firstinspires.ftc.team4100worlds.subsystem.Outtake;
import org.firstinspires.ftc.team4100worlds.subsystem.Plane;

public class ScrappyCore extends Robot {
    public final ScrappySettings.AllianceType ALLIANCE_TYPE;
    public final ScrappySettings.AllianceSide ALLIANCE_SIDE;

    // Subsystems
    public Follower m_drive;
    public Intake m_intake;
    public Lift m_lift;
    public Conveyor m_conveyor;
    public Outtake m_outtake;
    public Plane m_plane;

    public ScrappyCore(HardwareMap hardwareMap, ScrappySettings.AllianceType allianceType, ScrappySettings.AllianceSide allianceSide) {
        ALLIANCE_TYPE = allianceType;
        ALLIANCE_SIDE = allianceSide;

        // Schedule to clear cache continuously (manual mode)
        schedule(new BulkCacheHandler(hardwareMap));

        // Initialize subsystems
        m_drive = new Follower(hardwareMap);
        m_intake = new Intake(hardwareMap);
        m_lift = new Lift(hardwareMap);
        m_conveyor = new Conveyor(hardwareMap);
        m_outtake = new Outtake(hardwareMap);
        m_plane = new Plane(hardwareMap);
    }
}