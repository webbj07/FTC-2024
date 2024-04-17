package org.firstinspires.ftc.team4100worlds.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.team4100worlds.ScrappyConstants;
import org.firstinspires.ftc.team4100worlds.autonomous.ScrappyAutoBase;

// Assumes extender is down already
public class GrabFromStack extends SequentialCommandGroup {
    public GrabFromStack(ScrappyAutoBase base, Method method) {
        addCommands(
            (method == Method.Stack ?
                new SequentialCommandGroup(
                    new LocalizeWithStack(base),
                    new GotoStack(base)
                ) :
                new DriveToAprilTag(base, base.m_allianceType == ScrappyConstants.AllianceType.RED ? 8 : 9, false, new Pose2d(15.5))
            ),
            new InstantCommand(() -> {
                base.robot.m_intake.suck();
                base.robot.m_conveyor.up();
                base.robot.m_intake.grab();
            }),
            new WaitCommand(150),
            new InstantCommand(() -> base.robot.m_intake.backOne()),
            new WaitCommand(150),
            new InstantCommand(() -> base.robot.m_intake.backTwo()),
            new WaitCommand(150),
            new InstantCommand(() -> base.robot.m_intake.grab()),
            new WaitCommand(300)
        );
    }

    enum Method {
        Stack,
        AprilTag
    }
}