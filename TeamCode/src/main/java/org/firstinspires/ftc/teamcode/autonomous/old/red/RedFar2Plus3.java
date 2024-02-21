package org.firstinspires.ftc.teamcode.autonomous.old.red;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.commands.DriveToAprilTag;
import org.firstinspires.ftc.teamcode.commands.RunAction;
import org.firstinspires.ftc.teamcode.commands.RunFarCycle;

public class RedFar2Plus3 extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(-38, -61.75, Math.toRadians(90));
    private Action
            leftStackTraj, middleStackTraj, rightStackTraj,
            leftBackboardTraj, middleBackboardTraj, rightBackboardTraj,
            leftStackCycleTraj, middleStackCycleTraj, rightStackCycleTraj,
            backboardCycleTraj, endTraj;
    public RedFar2Plus3() {
        super(ScrappySettings.AllianceType.RED, ScrappySettings.AllianceSide.FAR, startingPose);
    }

    @Override
    public void initAuto() {
        // Left
        leftStackTraj = robot.m_drive.actionBuilder(startingPose)
                .stopAndAdd(robot.m_intake::lower)
                .splineToLinearHeading(new Pose2d(-35, -37, Math.toRadians(30)), 0)
                .stopAndAdd(() -> robot.m_intake.back())
                .waitSeconds(0.25)
                .stopAndAdd(() -> robot.m_intake.raise())
                .strafeToLinearHeading(new Vector2d(-51, -35.5), Math.toRadians(180))
                .build();

        leftBackboardTraj = robot.m_drive.actionBuilder(new Pose2d(-55.25, -35.5, Math.toRadians(180)))
                .lineToXConstantHeading(-50)
                .afterDisp(5, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .splineToConstantHeading(new Vector2d(-40, -59), 0)
                .endTrajectory()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(5, -59), 0)
                .splineToConstantHeading(new Vector2d(26.4, -59), 0)
                .afterDisp(16, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(480);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .splineToConstantHeading(new Vector2d(53, -41.5), 0)
                .build();

        leftStackCycleTraj = robot.m_drive.actionBuilder(new Pose2d(53, -41.5, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .strafeToLinearHeading(new Vector2d(36, -59), Math.toRadians(180))
                .endTrajectory()
                .splineToConstantHeading(new Vector2d(-35, -59), Math.toRadians(180))
                .afterDisp(3, robot.m_intake::spit)
                .afterDisp(8, robot.m_intake::stop)
                // y has to be an inch more cause of drift :/
                .splineToConstantHeading(new Vector2d(-47, -37), Math.toRadians(180))
                .stopAndAdd(robot.m_intake::lower)
                .build();

        // Middle
        middleStackTraj = robot.m_drive.actionBuilder(startingPose)
                .stopAndAdd(robot.m_intake::lower)
                .strafeTo(new Vector2d(-36, -35))
                .stopAndAdd(() -> robot.m_intake.back())
                .waitSeconds(0.25)
                .stopAndAdd(() -> robot.m_intake.raise())
                .lineToYConstantHeading(-40)
                .splineToSplineHeading(new Pose2d(-51, -35.5, Math.toRadians(180)), Math.toRadians(280))
                .build();

        middleBackboardTraj = robot.m_drive.actionBuilder(new Pose2d(-55.25, -35.5, Math.toRadians(180)))
                .lineToXConstantHeading(-50)
                .afterDisp(5, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .splineToConstantHeading(new Vector2d(-40, -59), 0)
                .endTrajectory()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(5, -59), 0)
                .splineToConstantHeading(new Vector2d(26.4, -59), 0)
                .afterDisp(20, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(480);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .splineToConstantHeading(new Vector2d(53, -35), 0)
                .build();

        middleStackCycleTraj = robot.m_drive.actionBuilder(new Pose2d(53, -35, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .strafeToLinearHeading(new Vector2d(36, -59), Math.toRadians(180))
                .endTrajectory()
                .splineToConstantHeading(new Vector2d(-35, -59), Math.toRadians(180))
                .afterDisp(3, robot.m_intake::spit)
                .afterDisp(8, robot.m_intake::stop)
                // y has to be an inch more cause of drift :/
                .splineToConstantHeading(new Vector2d(-47, -37), Math.toRadians(180))
                .stopAndAdd(robot.m_intake::lower)
                .build();

        // Right
        rightStackTraj = robot.m_drive.actionBuilder(startingPose)
                .stopAndAdd(robot.m_intake::lower)
                .splineToLinearHeading(new Pose2d(-45, -37, Math.toRadians(102)), Math.toRadians(190))
                .stopAndAdd(() -> robot.m_intake.back())
                .waitSeconds(0.25)
                .stopAndAdd(() -> robot.m_intake.raise())
                .splineToConstantHeading(new Vector2d(-45, -44), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-46, -35.5, Math.toRadians(180)), Math.toRadians(280))
                .build();

        rightBackboardTraj = robot.m_drive.actionBuilder(new Pose2d(-55.25, -35.5, Math.toRadians(180)))
                .lineToXConstantHeading(-50)
                .afterDisp(5, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .splineToConstantHeading(new Vector2d(-40, -59), 0)
                .endTrajectory()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(5, -59), 0)
                .splineToConstantHeading(new Vector2d(26.4, -59), 0)
                .afterDisp(20, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(480);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .splineToConstantHeading(new Vector2d(53, -28), 0)
                .build();

        rightStackCycleTraj = robot.m_drive.actionBuilder(new Pose2d(53, -28, Math.toRadians(180)))
                .afterDisp(5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .strafeToLinearHeading(new Vector2d(36, -59), Math.toRadians(180))
                .endTrajectory()
                .splineToConstantHeading(new Vector2d(-35, -59), Math.toRadians(180))
                .afterDisp(3, robot.m_intake::spit)
                .afterDisp(8, robot.m_intake::stop)
                // y has to be an inch more cause of drift :/
                .splineToConstantHeading(new Vector2d(-47, -37), Math.toRadians(180))
                .stopAndAdd(robot.m_intake::lower)
                .build();

        backboardCycleTraj = robot.m_drive.actionBuilder(new Pose2d(-55.25, -35.5, Math.toRadians(180)))
                .lineToXConstantHeading(-50)
                .afterDisp(5, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .strafeToLinearHeading(new Vector2d(-40, -59), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(5, -59), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(26.4, -59), 0)
                .afterDisp(20, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(700);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .splineToConstantHeading(new Vector2d(53, -41), 0)
                .build();

        endTraj = robot.m_drive.actionBuilder(new Pose2d(53, -41, Math.toRadians(180)))
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
            case RIGHT:
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
                        new DriveToAprilTag(this, 8, false, new PoseVelocity2d(new Vector2d(16, 0), 0)),
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
                        new InstantCommand(() -> vision.setActiveCamera(webcam2)),
                        new RunAction(stackCycleTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new WaitCommand(400),
                        new DriveToAprilTag(this, 8, false, new PoseVelocity2d(new Vector2d(16, 0), 0)),
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
                        new InstantCommand(() -> vision.setActiveCamera(webcam1)),
                        new RunAction(backboardCycleTraj),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(250),
                        new InstantCommand(() -> robot.m_lift.setRelativePosition(100)),
                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(750)),
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
