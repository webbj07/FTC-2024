package org.firstinspires.ftc.teamcode.autonomous.old.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.commands.DriveToAprilTag;
import org.firstinspires.ftc.teamcode.commands.RunAction;
import org.firstinspires.ftc.teamcode.commands.RunFarCycle;

public class BlueFar2Plus3Slow extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(-38, 61.75, Math.toRadians(270));
    private Action
            leftStackTraj, middleStackTraj, rightStackTraj,
            leftBackboardTraj, middleBackboardTraj, rightBackboardTraj,
            leftStackCycleTraj, middleStackCycleTraj, rightStackCycleTraj,
            backboardCycleTraj, endTraj;
    public BlueFar2Plus3Slow() {
        super(ScrappySettings.AllianceType.BLUE, ScrappySettings.AllianceSide.FAR, startingPose);
    }

    @Override
    public void initAuto() {
        // Left
        leftStackTraj = robot.m_drive.actionBuilder(startingPose)
                .stopAndAdd(robot.m_intake::lower)
                .splineToLinearHeading(new Pose2d(-35, 37, Math.toRadians(330)), 0)
                .stopAndAdd(() -> robot.m_intake.back())
                .waitSeconds(0.25)
                .stopAndAdd(() -> robot.m_intake.raise())
                .strafeToLinearHeading(new Vector2d(-51, 35.5), Math.toRadians(180))
                .build();

        leftBackboardTraj = robot.m_drive.actionBuilder(new Pose2d(-55.25, 35.5, Math.toRadians(180)))
                .lineToXConstantHeading(-50)
                .afterDisp(5, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .splineToConstantHeading(new Vector2d(-40, 59.5), 0)
                .endTrajectory()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(5, 59.5), 0)
                .splineToConstantHeading(new Vector2d(26.4, 59.5), 0)
                .afterDisp(16, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(480);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .splineToConstantHeading(new Vector2d(52, 41.5), 0)
                .build();

        leftStackCycleTraj = robot.m_drive.actionBuilder(new Pose2d(52, 41.5, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .strafeToLinearHeading(new Vector2d(36, 59.5), Math.toRadians(180))
                .endTrajectory()
                .splineToConstantHeading(new Vector2d(-35, 59.5), Math.toRadians(180))
                .afterDisp(3, robot.m_intake::spit)
                .afterDisp(8, robot.m_intake::stop)
                // y has to be an inch more cause of drift :/
                .splineToConstantHeading(new Vector2d(-47, 37), Math.toRadians(180))
                .stopAndAdd(robot.m_intake::lower)
                .build();

        // Middle
        middleStackTraj = robot.m_drive.actionBuilder(startingPose)
                .stopAndAdd(robot.m_intake::lower)
                .strafeTo(new Vector2d(-36, 35))
                .stopAndAdd(() -> robot.m_intake.back())
                .waitSeconds(0.25)
                .stopAndAdd(() -> robot.m_intake.raise())
                .lineToYConstantHeading(40)
                .splineToSplineHeading(new Pose2d(-51, 35.5, Math.toRadians(180)), Math.toRadians(280))
                .build();

        middleBackboardTraj = robot.m_drive.actionBuilder(new Pose2d(-55.25, 35.5, Math.toRadians(180)))
                .lineToXConstantHeading(-50)
                .afterDisp(5, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .splineToConstantHeading(new Vector2d(-40, 59.5), 0)
                .endTrajectory()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(5, 59.5), 0)
                .splineToConstantHeading(new Vector2d(26.4, 59.5), 0)
                .afterDisp(20, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(480);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .splineToConstantHeading(new Vector2d(52, 35), 0)
                .build();

        middleStackCycleTraj = robot.m_drive.actionBuilder(new Pose2d(52, 35, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .strafeToLinearHeading(new Vector2d(36, 59.5), Math.toRadians(180))
                .endTrajectory()
                .splineToConstantHeading(new Vector2d(-35, 59.5), Math.toRadians(180))
                .afterDisp(3, robot.m_intake::spit)
                .afterDisp(8, robot.m_intake::stop)
                // y has to be an inch more cause of drift :/
                .splineToConstantHeading(new Vector2d(-47, 37), Math.toRadians(180))
                .stopAndAdd(robot.m_intake::lower)
                .build();

        // Right
        rightStackTraj = robot.m_drive.actionBuilder(startingPose)
                .stopAndAdd(robot.m_intake::lower)
                .splineToLinearHeading(new Pose2d(-45, 37, Math.toRadians(258)), Math.toRadians(190))
                .stopAndAdd(() -> robot.m_intake.back())
                .waitSeconds(0.25)
                .stopAndAdd(() -> robot.m_intake.raise())
                .splineToConstantHeading(new Vector2d(-45, 44), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-51, 35.5, Math.toRadians(180)), Math.toRadians(280))
                .build();

        rightBackboardTraj = robot.m_drive.actionBuilder(new Pose2d(-55.25, 35.5, Math.toRadians(180)))
                .lineToXConstantHeading(-50)
                .afterDisp(5, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .splineToConstantHeading(new Vector2d(-40, 59.5), 0)
                .endTrajectory()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(5, 59.5), 0)
                .splineToConstantHeading(new Vector2d(26.4, 59.5), 0)
                .afterDisp(20, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(480);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .splineToConstantHeading(new Vector2d(52, 28), 0)
                .build();

        rightStackCycleTraj = robot.m_drive.actionBuilder(new Pose2d(52, 28, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .strafeToLinearHeading(new Vector2d(36, 59.5), Math.toRadians(180))
                .endTrajectory()
                .splineToConstantHeading(new Vector2d(-35, 59.5), Math.toRadians(180))
                .afterDisp(3, robot.m_intake::spit)
                .afterDisp(8, robot.m_intake::stop)
                // y has to be an inch more cause of drift :/
                .splineToConstantHeading(new Vector2d(-47, 37), Math.toRadians(180))
                .stopAndAdd(robot.m_intake::lower)
                .build();

        backboardCycleTraj = robot.m_drive.actionBuilder(new Pose2d(-55.25, 35.5, Math.toRadians(180)))
                .lineToXConstantHeading(-50)
                .afterDisp(5, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .strafeToLinearHeading(new Vector2d(-40, 59.5), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(5, 59.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(26.4, 59.5), 0)
                .afterDisp(20, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(700);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .splineToConstantHeading(new Vector2d(52.5, 41), 0)
                .build();

        endTraj = robot.m_drive.actionBuilder(new Pose2d(52.5, 41, Math.toRadians(180)))
                .lineToX(43)
                .afterDisp(3, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .build();
    }

    @Override
    public void startAuto() {
        Action stackTraj, backboardTraj, stackCycleTraj;

        switch (detectionResult) {
            case LEFT:
                stackTraj = leftStackTraj;
                backboardTraj = leftBackboardTraj;
                stackCycleTraj = leftStackCycleTraj;
                break;
            case MIDDLE:
                stackTraj = middleStackTraj;
                backboardTraj = middleBackboardTraj;
                stackCycleTraj = middleStackCycleTraj;
                break;
            default:
                stackTraj = rightStackTraj;
                backboardTraj = rightBackboardTraj;
                stackCycleTraj = rightStackCycleTraj;
                break;
        }

        schedule(
                new SequentialCommandGroup(
                        new RunAction(stackTraj),
                        new InstantCommand(robot.m_intake::lower),
                        new WaitCommand(400),
                        new DriveToAprilTag(this, 9, false, new PoseVelocity2d(new Vector2d(15, 0), 0)),
                        new InstantCommand(() -> {
                            robot.m_intake.suck();
                            robot.m_conveyor.up();
                            robot.m_intake.grab();
                        }),
                        new InstantCommand(() -> vision.setActiveCamera(webcam1)),
                        new RunAction(backboardTraj),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(100),
                        new InstantCommand(() -> robot.m_lift.setRelativePosition(100)),
                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(560)),
                        new RunFarCycle(this, backboardCycleTraj, stackCycleTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
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
