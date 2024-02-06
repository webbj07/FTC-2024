package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.vision.AprilTagLocalization.getBackboardIdFromDetection;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ScrappySettings.AllianceType;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.interfaces.PropDetector.DetectionResult;

public class RunFarCycle extends SequentialCommandGroup {
    public RunFarCycle(ScrappyAutoBase base, DetectionResult detectionResult, AllianceType alliance, Action backboardTraj, Action stackTraj) {
        addCommands(
                new InstantCommand(() -> base.vision.setActiveCamera(base.webcam2)),
                new RunAction(stackTraj),
                new WaitCommand(125),
                new InstantCommand(base.robot.m_intake::exDown),
                new WaitCommand(250),
                new DriveToAprilTag(base, alliance == AllianceType.RED ? 8 : 9, false, 16),
                new InstantCommand(() -> {
                    base.robot.m_intake.suck();
                    base.robot.m_conveyor.up();
                    base.robot.m_intake.grab();
                }),
                new WaitCommand(1000),
                new SequentialCommandGroup(
                        new InstantCommand(() -> base.robot.m_intake.backOne()),
                        new WaitCommand(400),
                        new InstantCommand(() -> base.robot.m_intake.backTwo()),
                        new WaitCommand(400),
                        new InstantCommand(() -> base.robot.m_intake.grab())
                ),
                new WaitCommand(1000),
                new InstantCommand(() -> base.vision.setActiveCamera(base.webcam1)),
                new RunAction(backboardTraj),
                new InstantCommand(() -> {
                    base.robot.m_outtake.setExtPos(0.53);
                    base.robot.m_lift.setRelativePosition(900);
                }),
                new DriveToAprilTag(base, getBackboardIdFromDetection(alliance, detectionResult), true, 12),
                new SequentialCommandGroup(
                        new InstantCommand(base.robot.m_outtake::drop),
                        new WaitCommand(300)
                )
        );
    }
}
