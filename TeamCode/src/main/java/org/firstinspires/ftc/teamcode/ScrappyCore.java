package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.commands.BulkCacheHandler;
import org.firstinspires.ftc.teamcode.subsystem.Conveyor;
import org.firstinspires.ftc.teamcode.subsystem.Dropper;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Plane;

public class ScrappyCore extends Robot {
    public final ScrappySettings.AllianceType ALLIANCE_TYPE;
    public final ScrappySettings.AllianceSide ALLIANCE_SIDE;

    // Subsystems
    public MecanumDrive m_drive;
    public Intake m_intake;
    public Lift m_lift;
    public Conveyor m_conveyor;
    public Dropper m_dropper;
    public Plane m_plane;

    public ScrappyCore(HardwareMap hardwareMap, ScrappySettings.AllianceType allianceType, ScrappySettings.AllianceSide allianceSide, Pose2d startPose) {
        ALLIANCE_TYPE = allianceType;
        ALLIANCE_SIDE = allianceSide;

        // Schedule to clear cache continuously (manual mode)
        schedule(new BulkCacheHandler(hardwareMap));

        // Initialize subsystems
        m_drive = new MecanumDrive(hardwareMap, startPose);
        m_intake = new Intake(hardwareMap);
        m_lift = new Lift(hardwareMap);
        m_conveyor = new Conveyor(hardwareMap);
        m_dropper = new Dropper(hardwareMap);
        m_plane = new Plane(hardwareMap);
    }
}