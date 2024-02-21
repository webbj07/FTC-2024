package org.firstinspires.ftc.teamcode.autonomous.old.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.commands.LocalizeWithAprilTag;
import org.firstinspires.ftc.teamcode.commands.RunAction;

public class BlueClose2Plus4AT extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    private Action leftDetectionTraj, middleDetectionTraj, rightDetectionTraj,
            leftAprilTagTraj, middleAprilTagTraj, rightAprilTagTraj,
            stackTraj,
            leftBackboardTraj, middleBackboardTraj, rightBackboardTraj,
            leftAprilTagTraj2, middleAprilTagTraj2, rightAprilTagTraj2,
            stackTraj2,
            leftBackboardTraj2, middleBackboardTraj2, rightBackboardTraj2,
            leftEndTraj, middleEndTraj, rightEndTraj;

    public BlueClose2Plus4AT() {
        super(ScrappySettings.AllianceType.BLUE, ScrappySettings.AllianceSide.CLOSE, startingPose);
    }

    @Override
    public void initAuto() {
        TrajectoryActionBuilder backboardTrajBuilder = robot.m_drive.actionBuilder(new Pose2d(-56, 11.5, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .strafeToConstantHeading(new Vector2d(30, 11))
                .afterDisp(10, () -> {
                    robot.m_intake.stop();
                    robot.m_conveyor.stop();
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(800);
                });

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
        leftAprilTagTraj = robot.m_drive.actionBuilder(new Pose2d(52.75, 40.25, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .build();
        leftBackboardTraj = backboardTrajBuilder.splineToConstantHeading(new Vector2d(52.75, 29), 0).build();
        leftAprilTagTraj2 = robot.m_drive.actionBuilder(new Pose2d(52.75, 29, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .build();
        leftBackboardTraj2 = backboardTrajBuilder.splineToConstantHeading(new Vector2d(52.75, 29), 0).build();
        leftEndTraj = robot.m_drive.actionBuilder(new Pose2d(52.75, 29, Math.toRadians(180)))
                .lineToX(44.5)
                .afterDisp(3, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
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
                .strafeToLinearHeading(new Vector2d(52.75, 33), Math.toRadians(180))
                .build();
        middleAprilTagTraj = robot.m_drive.actionBuilder(new Pose2d(52.75, 33, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .build();
        middleBackboardTraj = backboardTrajBuilder.splineToConstantHeading(new Vector2d(52.75, 29), 0).build();
        middleAprilTagTraj2 = robot.m_drive.actionBuilder(new Pose2d(52.75, 29, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .build();
        middleBackboardTraj2 = backboardTrajBuilder.splineToConstantHeading(new Vector2d(52.75, 29), 0).build();
        middleEndTraj = robot.m_drive.actionBuilder(new Pose2d(52.75, 29, Math.toRadians(180)))
                .lineToX(44.5)
                .afterDisp(3, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
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
        rightAprilTagTraj = robot.m_drive.actionBuilder(new Pose2d(52.75, 26, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .build();
        rightBackboardTraj = backboardTrajBuilder.splineToConstantHeading(new Vector2d(52.25, 34), 0).build();
        rightAprilTagTraj2 = robot.m_drive.actionBuilder(new Pose2d(52.25, 34, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .build();
        rightBackboardTraj2 = backboardTrajBuilder.splineToConstantHeading(new Vector2d(52.25, 31), 0).build();
        rightEndTraj = robot.m_drive.actionBuilder(new Pose2d(52.25, 31, Math.toRadians(180)))
                .lineToX(44.5)
                .afterDisp(3, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .build();

        // Other
        stackTraj = robot.m_drive.actionBuilder(new Pose2d(-40, 10.5, Math.toRadians(150)))
                .turnTo(Math.PI)
                .stopAndAdd(robot.m_intake::lower)
                .strafeToLinearHeading(new Vector2d(-56, 11.5), Math.toRadians(180))
                .build();

        stackTraj2 = robot.m_drive.actionBuilder(new Pose2d(-40, 10.5, Math.toRadians(150)))
                .turnTo(Math.PI)
                .stopAndAdd(robot.m_intake::lower)
                .strafeToLinearHeading(new Vector2d(-56, 11.5), Math.toRadians(180))
                .build();
    }

    @Override
    public void startAuto() {
        Action spikeAndBackboardTraj, aprilTagTraj, backboardTraj, aprilTagTraj2, backboardTraj2, endTraj;

        switch (detectionResult) {
            case LEFT:
                spikeAndBackboardTraj = leftDetectionTraj;
                aprilTagTraj = leftAprilTagTraj;
                backboardTraj = leftBackboardTraj;
                aprilTagTraj2 = leftAprilTagTraj2;
                backboardTraj2 = leftBackboardTraj2;
                endTraj = leftEndTraj;
                break;
            case MIDDLE:
                spikeAndBackboardTraj = middleDetectionTraj;
                aprilTagTraj = middleAprilTagTraj;
                backboardTraj = middleBackboardTraj;
                aprilTagTraj2 = middleAprilTagTraj2;
                backboardTraj2 = middleBackboardTraj2;
                endTraj = middleEndTraj;
                break;
            default:
                spikeAndBackboardTraj = rightDetectionTraj;
                aprilTagTraj = rightAprilTagTraj;
                backboardTraj = rightBackboardTraj;
                aprilTagTraj2 = rightAprilTagTraj2;
                backboardTraj2 = rightBackboardTraj2;
                endTraj = rightEndTraj;
                break;
        }

        schedule(
                new SequentialCommandGroup(
                        new RunAction(spikeAndBackboardTraj),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(100),

                        // cycle 1
                        new InstantCommand(() -> vision.setActiveCamera(webcam2)),
                        new RunAction(aprilTagTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new WaitCommand(500),
                        new LocalizeWithAprilTag(this, false, new Pose2d(0, -1, 0)),
                        new RunAction(stackTraj),
                        new InstantCommand(() -> {
                            robot.m_intake.suck();
                            robot.m_conveyor.up();
                            robot.m_intake.grab();
                        }),
                        new WaitCommand(250),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.m_intake.backOne()),
                                new WaitCommand(400),
                                new InstantCommand(() -> robot.m_intake.backTwo()),
                                new WaitCommand(400),
                                new InstantCommand(() -> robot.m_intake.grab())
                        ),
                        new RunAction(backboardTraj),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(300),

                        // cycle 2
                        new RunAction(aprilTagTraj2).alongWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new WaitCommand(500),
                        new LocalizeWithAprilTag(this, false, new Pose2d(0, -1, 0)),
                        new RunAction(stackTraj2),
                        new InstantCommand(() -> {
                            robot.m_intake.suck();
                            robot.m_conveyor.up();
                            robot.m_intake.grab();
                        }),
                        new WaitCommand(250),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.m_intake.backOne()),
                                new WaitCommand(400),
                                new InstantCommand(() -> robot.m_intake.backTwo()),
                                new WaitCommand(400),
                                new InstantCommand(() -> robot.m_intake.grab())
                        ),
                        new RunAction(backboardTraj2),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(300),

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
