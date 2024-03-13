package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;

public class RunFarCycle extends SequentialCommandGroup {
    public RunFarCycle(ScrappyAutoBase base, Action backboardTraj, Action stackTraj) {
        addCommands(
                new InstantCommand(() -> base.vision.setActiveCamera(base.webcam2)),
                new RunAction(stackTraj),
                new WaitCommand(400),
                new DriveToAprilTag(base, 9, false, new PoseVelocity2d(new Vector2d(15, 0), 0)),
                new InstantCommand(() -> {
                    base.robot.m_intake.suck();
                    base.robot.m_conveyor.up();
                    base.robot.m_intake.grab();
                }),
                new WaitCommand(250),
                new SequentialCommandGroup(
                        new InstantCommand(() -> base.robot.m_intake.backOne()),
                        new WaitCommand(400),
                        new InstantCommand(() -> base.robot.m_intake.backTwo()),
                        new WaitCommand(400),
                        new InstantCommand(() -> base.robot.m_intake.grab())
                ),
                new InstantCommand(() -> base.vision.setActiveCamera(base.webcam1)),
                new RunAction(backboardTraj),
                new InstantCommand(base.robot.m_outtake::drop),
                new WaitCommand(250),
                new InstantCommand(() -> base.robot.m_lift.setRelativePosition(100)),
                new WaitUntilCommand(() -> base.robot.m_lift.isWithinTolerance(750))
        );
    }
}
