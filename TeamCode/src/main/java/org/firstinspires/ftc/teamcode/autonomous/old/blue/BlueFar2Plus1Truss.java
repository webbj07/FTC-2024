package org.firstinspires.ftc.teamcode.autonomous.old.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.teamcode.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.teamcode.commands.LocalizeWithAprilTag;
import org.firstinspires.ftc.teamcode.commands.RunAction;

@Autonomous
public class BlueFar2Plus1Truss extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(-39, 61.75, Math.toRadians(270));
    private Action leftStackTraj, middleStackTraj, rightStackTraj, gotoStackTraj;
    public BlueFar2Plus1Truss() {
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
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), 0)
                .build();

        // Middle
        middleStackTraj = robot.m_drive.actionBuilder(startingPose)
                .stopAndAdd(robot.m_intake::lower)
                .strafeTo(new Vector2d(-36, 35))
                .stopAndAdd(() -> robot.m_intake.back())
                .waitSeconds(0.25)
                .stopAndAdd(() -> robot.m_intake.raise())
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-55, 36))
                .strafeToConstantHeading(new Vector2d(-55, 11.5))
                .strafeToLinearHeading(new Vector2d(-40, 11.5), Math.toRadians(150))
                .build();

        // Right
        rightStackTraj = robot.m_drive.actionBuilder(startingPose)
                .strafeTo(new Vector2d(-35, 58))
                .strafeTo(new Vector2d(-35, 12))
                .stopAndAdd(robot.m_intake::lower)
                .strafeToLinearHeading(new Vector2d(-38, 16), Math.toRadians(130))
                .stopAndAdd(() -> robot.m_intake.back())
                .waitSeconds(0.25)
                .stopAndAdd(() -> robot.m_intake.raise())
                .strafeToLinearHeading(new Vector2d(-40, 11.5), Math.toRadians(150))
                .build();

        gotoStackTraj = robot.m_drive.actionBuilder(new Pose2d(-40, 10.5, Math.toRadians(150)))
                .turnTo(Math.PI)
                .stopAndAdd(robot.m_intake::lower)
                .strafeToLinearHeading(new Vector2d(-56, 11.5), Math.toRadians(180))
                .build();
    }

    @Override
    public void startAuto() {
        Action stackTraj, backboardTraj, endTraj;

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
                    robot.m_lift.setRelativePosition(550);
                });

        TurnConstraints turnConstraintsOverride = new TurnConstraints(
                Math.toRadians(300), -Math.toRadians(300), Math.toRadians(300));
        switch (detectionResult) {
            case LEFT:
                stackTraj = leftStackTraj;
                backboardTraj = backboardTrajBuilder.splineToConstantHeading(new Vector2d(52.5, 40.25), 0).build();
                endTraj = robot.m_drive.actionBuilder(new Pose2d(52.5, 40.25, Math.toRadians(180)))
                        .afterDisp(3, () -> {
                            robot.m_lift.toInitial();
                            robot.m_outtake.back();
                        })
                        .splineToLinearHeading(new Pose2d(48, 12.5, Math.toRadians(90)), 0)
                        .endTrajectory()
//                        .turn(8 * Math.PI, turnConstraintsOverride)
                        .turnTo(Math.PI)
                        .build();
                break;
            case MIDDLE:
                stackTraj = middleStackTraj;
                backboardTraj = backboardTrajBuilder.splineToConstantHeading(new Vector2d(52.5, 33), 0).build();
                endTraj = robot.m_drive.actionBuilder(new Pose2d(52.5, 33, Math.toRadians(180)))
                        .afterDisp(3, () -> {
                            robot.m_lift.toInitial();
                            robot.m_outtake.back();
                        })
                        .splineToLinearHeading(new Pose2d(48, 12.5, Math.toRadians(90)), 0)
                        .endTrajectory()
//                        .turn(8 * Math.PI, turnConstraintsOverride)
                        .turnTo(Math.PI)
                        .build();
                break;
            default:
                stackTraj = rightStackTraj;
                backboardTraj = backboardTrajBuilder.splineToConstantHeading(new Vector2d(52.5, 27.5), 0).build();
                endTraj = robot.m_drive.actionBuilder(new Pose2d(52.5, 27.5, Math.toRadians(180)))
                        .afterDisp(3, () -> {
                            robot.m_lift.toInitial();
                            robot.m_outtake.back();
                        })
                        .splineToLinearHeading(new Pose2d(48, 12.5, Math.toRadians(90)), 0)
                        .endTrajectory()
//                        .turn(8 * Math.PI, turnConstraintsOverride)
                        .turnTo(Math.PI)
                        .build();
                break;
        }

        schedule(
                new SequentialCommandGroup(
                        new RunAction(stackTraj),
                        new WaitCommand(500),
                        new LocalizeWithAprilTag(this, false, new Pose2d(0, -1, 0)),
                        new RunAction(gotoStackTraj),
                        new WaitCommand(400),
                        new InstantCommand(() -> {
                            robot.m_intake.suck();
                            robot.m_conveyor.up();
                            robot.m_intake.grab();
                        }),
                        new WaitCommand(250),
                        new RunAction(backboardTraj),
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
