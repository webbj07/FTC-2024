package org.firstinspires.ftc.teamcode.autonomous.old.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.commands.RunAction;

public class BlueClose2Plus0 extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    private Action leftDetectionTraj, middleDetectionTraj, rightDetectionTraj,
            leftEndTraj, middleEndTraj, rightEndTraj;

    public BlueClose2Plus0() {
        super(ScrappySettings.AllianceType.BLUE, ScrappySettings.AllianceSide.CLOSE, startingPose);
    }

    @Override
    public void initAuto() {
        // Left
        leftDetectionTraj = robot.m_drive.actionBuilder(startingPose)
                .stopAndAdd(robot.m_intake::lower)
                .splineToLinearHeading(new Pose2d(30, 37, Math.toRadians(230)), Math.toRadians(270))
                .stopAndAdd(() -> {
                    robot.m_intake.back();
                    robot.m_intake.raise();
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(390);
                })
                .strafeToLinearHeading(new Vector2d(52.75, 40.25), Math.toRadians(180))
                .build();
        leftEndTraj = robot.m_drive.actionBuilder(new Pose2d(52.75, 40.25, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .splineToConstantHeading(new Vector2d(50, 60), 0)
                .splineToConstantHeading(new Vector2d(59.5, 60), 0)
                .build();

        // Middle
        middleDetectionTraj = robot.m_drive.actionBuilder(startingPose)
                .stopAndAdd(robot.m_intake::lower)
                .strafeTo(new Vector2d(11.5, 35.5))
                .stopAndAdd(() -> {
                    robot.m_intake.back();
                    robot.m_intake.raise();
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(390);
                })
                .strafeToLinearHeading(new Vector2d(52.75, 33.5), Math.toRadians(180))
                .build();
        middleEndTraj = robot.m_drive.actionBuilder(new Pose2d(52.75, 33.5, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .splineToConstantHeading(new Vector2d(50, 60), 0)
                .splineToConstantHeading(new Vector2d(59.5, 60), 0)
                .build();

        // Right
        rightDetectionTraj = robot.m_drive.actionBuilder(startingPose)
                .stopAndAdd(robot.m_intake::lower)
                .splineToLinearHeading(new Pose2d(10, 37, Math.toRadians(210)), Math.PI)
                .stopAndAdd(() -> {
                    robot.m_intake.back();
                    robot.m_intake.raise();
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(390);
                })
                .strafeToLinearHeading(new Vector2d(52.75, 26), Math.toRadians(180))
                .build();
        rightEndTraj = robot.m_drive.actionBuilder(new Pose2d(52.25, 26, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .splineToConstantHeading(new Vector2d(50, 60), 0)
                .splineToConstantHeading(new Vector2d(59.5, 60), 0)
                .build();
    }

    @Override
    public void startAuto() {
        Action spikeAndBackboardTraj, endTraj;

        switch (detectionResult) {
            case LEFT:
                spikeAndBackboardTraj = leftDetectionTraj;
                endTraj = leftEndTraj;
                break;
            case MIDDLE:
                spikeAndBackboardTraj = middleDetectionTraj;
                endTraj = middleEndTraj;
                break;
            default:
                spikeAndBackboardTraj = rightDetectionTraj;
                endTraj = rightEndTraj;
                break;
        }

        schedule(
                new SequentialCommandGroup(
                        new RunAction(spikeAndBackboardTraj),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(250),
                        new RunAction(endTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        )

                )
        );
    }
}
