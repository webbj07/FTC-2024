package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.commands.DriveToAprilTag;
import org.firstinspires.ftc.teamcode.commands.RunAction;
import org.firstinspires.ftc.teamcode.commands.RunFarCycle;

public class BlueFar2PlusOne extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(-39, 61.75, Math.toRadians(90.00));
    private Action stackTraj, backboardTraj, stackCycleTraj, backboardTraj2, endTraj;
    public BlueFar2PlusOne() {
        super(ScrappySettings.AllianceType.BLUE, ScrappySettings.AllianceSide.FAR, startingPose);
    }

    @Override
    public void initAuto() {
        stackTraj = robot.m_drive.actionBuilder(startingPose)
                .strafeTo(new Vector2d(-35.23, 33.51))
                .strafeToLinearHeading(new Vector2d(-52, 36), Math.toRadians(180.00))
                .build();

        backboardTraj = robot.m_drive.actionBuilder(new Pose2d(-58.5, 35.5, Math.toRadians(180.00)))
                .afterDisp(5, () -> {
                    robot.m_intake.exUp();
                    robot.m_intake.back();
                })
                .afterDisp(45, () -> robot.m_intake.stop())
                .lineToXConstantHeading(-48)
                .splineToConstantHeading(new Vector2d(-35, 56), 0)
                .splineToConstantHeading(new Vector2d(23, 56), 0)
                .splineToConstantHeading(new Vector2d(36, 31), 0)
                .build();

        backboardTraj2 = robot.m_drive.actionBuilder(new Pose2d(-58.5, 35.5, Math.toRadians(180.00)))
                .afterDisp(5, () -> {
                    robot.m_intake.exUp();
                    robot.m_intake.back();
                })
                .afterDisp(45, () -> robot.m_intake.stop())
                .lineToXConstantHeading(-48)
                .splineToConstantHeading(new Vector2d(-35, 56), 0)
                .splineToConstantHeading(new Vector2d(23, 56), 0)
                .splineToConstantHeading(new Vector2d(36, 28), 0)
                .build();

        stackCycleTraj = robot.m_drive.actionBuilder(new Pose2d(49, 35.5, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                    robot.m_outtake.setExtPos(0.155);
                })
                .lineToXConstantHeading(35.5)
                .splineToConstantHeading(new Vector2d(22.5, 57.9), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-35, 57.9), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-52, 38), Math.toRadians(180))
                .build();

        endTraj = robot.m_drive.actionBuilder(new Pose2d(48, 35.5, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                    robot.m_outtake.setExtPos(0.155);
                })
                .strafeTo(new Vector2d(42, 58))
                .build();
    }

    @Override
    public void startAuto() {
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(robot.m_intake::exUp),
                        new InstantCommand(robot.m_outtake::exLow),
                        new RunAction(stackTraj),
                        new InstantCommand(robot.m_intake::exDown),
                        new WaitCommand(350),
                        new DriveToAprilTag(this, 9, false, 16),
                        new InstantCommand(() -> {
                            robot.m_intake.suck();
                            robot.m_conveyor.up();
                            robot.m_intake.grab();
                        }),
                        new WaitCommand(1000),
                        new InstantCommand(() -> vision.setActiveCamera(webcam1)),
                        new RunAction(backboardTraj),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> {
                                    robot.m_outtake.setExtPos(0.53);
                                    robot.m_lift.setRelativePosition(600);
                                }),
                                new DriveToAprilTag(this, 2, true, 11)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(robot.m_outtake::drop),
                                new WaitCommand(300)
                        ),
                        new RunFarCycle(this, detectionResult, m_allianceType, backboardTraj2, stackCycleTraj),
                        new RunAction(endTraj)
                )
        );
    }
}
