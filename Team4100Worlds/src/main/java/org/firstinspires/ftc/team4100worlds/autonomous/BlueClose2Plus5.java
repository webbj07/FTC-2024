package org.firstinspires.ftc.team4100worlds.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.team4100worlds.ScrappyConstants;
import org.firstinspires.ftc.team4100worlds.commands.FollowPath;
import org.firstinspires.ftc.team4100worlds.commands.GotoStack;
import org.firstinspires.ftc.team4100worlds.commands.HoldPoint;
import org.firstinspires.ftc.team4100worlds.commands.LocalizeWithStack;
import org.firstinspires.ftc.team4100worlds.commands.ProfiledLiftCommand;
import org.firstinspires.ftc.team4100worlds.commands.WaitForReachedTValue;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierCurve;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierPoint;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;

@Autonomous
public class BlueClose2Plus5 extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    private PathChain middleSpikeMark, middleBackboardTraj, middleStackTraj, middleBackboardStackTraj, middleStack2Traj, middleBackboardStack2Traj, middleStack3Traj, middleBackboardStack3Traj, middleEndTraj;

    public BlueClose2Plus5() {
        super(ScrappyConstants.AllianceType.BLUE, ScrappyConstants.AllianceSide.CLOSE, startingPose);
    }

    @Override
    public void initAuto() {
        // Middle
        middleSpikeMark = robot.m_drive.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startingPose), new Point(12, 37.5, Point.CARTESIAN))))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(220))
                .build();

        middleBackboardTraj = robot.m_drive.pathBuilder()
                .addPath(new Path(new BezierLine(middleSpikeMark.getPath(0).getLastControlPoint(), new Point(52.5, 27, Point.CARTESIAN))))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.PI, 0.7)
                .build();

        middleStackTraj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addTemporalCallback(1, () -> vision.setProcessorEnabled(stackProcessor, true))
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-45, 11.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStackTraj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(52.2, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.1, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .addParametricCallback(0.5, () -> {
                    robot.m_intake.stop();
                    robot.m_conveyor.stop();
                    robot.m_outtake.extend();
                })
                .build();
        middleStack2Traj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addTemporalCallback(1, () -> vision.setProcessorEnabled(stackProcessor, true))
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-45, 11.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStack2Traj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(52.2, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.1, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .addParametricCallback(0.5, () -> {
                    robot.m_intake.stop();
                    robot.m_conveyor.stop();
                    robot.m_outtake.extend();
                })
                .build();
        middleStack3Traj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addTemporalCallback(1, () -> vision.setProcessorEnabled(stackProcessor, true))
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-65, 11.75, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.8, () -> {
                    robot.m_intake.suck();
                    robot.m_conveyor.up();
                })
                .build();

        middleBackboardStack3Traj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(52.2, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.1, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .addParametricCallback(0.5, () -> {
                    robot.m_intake.stop();
                    robot.m_conveyor.stop();
                    robot.m_outtake.extend();
                })
                .build();
        middleEndTraj = robot.m_drive.pathBuilder()
                .addPath(new Path(new BezierLine(middleBackboardStackTraj.getPath(0).getLastControlPoint(), new Point(47, 35, Point.CARTESIAN))))
                .setConstantHeadingInterpolation(Math.PI)
                .build();
    }

    @Override
    public void startAuto() {
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(robot.m_intake::lower),
                        new WaitCommand(200),
                        new FollowPath(robot.m_drive, middleSpikeMark),
                        new InstantCommand(() -> {
                            robot.m_intake.back();
                            robot.m_outtake.extend();
                        }),
                        new WaitCommand(200),
                        new InstantCommand(robot.m_intake::raise),
                        new FollowPath(robot.m_drive, middleBackboardTraj).alongWith(new ProfiledLiftCommand(robot.m_lift, 430)),
                        new HoldPoint(robot.m_drive, new BezierPoint(middleBackboardTraj.getPath(0).getLastControlPoint()), Math.PI),
                        new WaitCommand(400),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(500),

                        new FollowPath(robot.m_drive, middleStackTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new WaitForReachedTValue(robot.m_drive, 0.05),
                                        new InstantCommand(robot.m_outtake::back),
                                        new ProfiledLiftCommand(robot.m_lift, 0),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new LocalizeWithStack(this),
                        new InstantCommand(() -> vision.setProcessorEnabled(stackProcessor, false)),
                        new HoldPoint(robot.m_drive, new BezierPoint(middleStackTraj.getPath(1).getLastControlPoint()), Math.PI),
                        new InstantCommand(robot.m_intake::lower),
                        new WaitCommand(300),
                        new GotoStack(this),
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
                                new InstantCommand(() -> robot.m_intake.grab()),
                                new WaitCommand(250)
                        ),
                        new FollowPath(robot.m_drive, middleBackboardStackTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new WaitForReachedTValue(robot.m_drive, 0.5),
                                        new ProfiledLiftCommand(robot.m_lift, 900)
                                )
                        ),
                        new WaitCommand(200),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(500),

                        new FollowPath(robot.m_drive, middleStack2Traj).alongWith(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> vision.setProcessorEnabled(stackProcessor, true)),
                                        new WaitCommand(200),
                                        new WaitForReachedTValue(robot.m_drive, 0.05),
                                        new InstantCommand(robot.m_outtake::back),
                                        new ProfiledLiftCommand(robot.m_lift, 0),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new LocalizeWithStack(this),
                        new InstantCommand(() -> vision.setProcessorEnabled(stackProcessor, false)),
                        new HoldPoint(robot.m_drive, new BezierPoint(middleStackTraj.getPath(1).getLastControlPoint()), Math.PI),
                        new InstantCommand(robot.m_intake::lower),
                        new WaitCommand(300),
                        new GotoStack(this),
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
                                new InstantCommand(() -> robot.m_intake.grab()),
                                new WaitCommand(250)
                        ),
                        new FollowPath(robot.m_drive, middleBackboardStack2Traj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new WaitForReachedTValue(robot.m_drive, 0.5),
                                        new ProfiledLiftCommand(robot.m_lift, 900)
                                )
                        ),
                        new WaitCommand(200),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(500),

                        new FollowPath(robot.m_drive, middleStack3Traj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new WaitForReachedTValue(robot.m_drive, 0.05),
                                        new InstantCommand(robot.m_outtake::back),
                                        new ProfiledLiftCommand(robot.m_lift, 0),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new WaitForReachedTValue(robot.m_drive, 0.7),
                                        new InstantCommand(robot.m_intake::suck),
                                        new InstantCommand(robot.m_conveyor::up)
                                )
                        ),
                        new WaitCommand(400),
                        new FollowPath(robot.m_drive, middleBackboardStack3Traj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new WaitForReachedTValue(robot.m_drive, 0.5),
                                        new ProfiledLiftCommand(robot.m_lift, 1000)
                                )
                        ),
                        new WaitCommand(200),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(500),

                        new FollowPath(robot.m_drive, middleEndTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new InstantCommand(robot.m_outtake::back),
                                        new WaitForReachedTValue(robot.m_drive, 0.2),
                                        new ProfiledLiftCommand(robot.m_lift, 0),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        )
                )
        );
    }
}
