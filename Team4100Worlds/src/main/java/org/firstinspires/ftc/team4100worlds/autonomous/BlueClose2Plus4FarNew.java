package org.firstinspires.ftc.team4100worlds.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team4100worlds.FieldConstants;
import org.firstinspires.ftc.team4100worlds.ScrappyConstants;
import org.firstinspires.ftc.team4100worlds.commands.DriveToAprilTag;
import org.firstinspires.ftc.team4100worlds.commands.FollowPath;
import org.firstinspires.ftc.team4100worlds.commands.HoldPoint;
import org.firstinspires.ftc.team4100worlds.commands.ProfiledLiftCommand;
import org.firstinspires.ftc.team4100worlds.commands.WaitForReachedTValue;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierCurve;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierPoint;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;

@Autonomous
public class BlueClose2Plus4FarNew extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(FieldConstants.TileLength - ScrappyConstants.CHASSIS_WIDTH / 2 - 1, FieldConstants.MaxWallCoordinate - ScrappyConstants.CHASSIS_LENGTH / 2, Math.toRadians(270));
    private PathChain middleSpikeMark, middleBackboardTraj, middleStackTraj, middleBackboardStackTraj, middleStack2Traj, middleBackboardStack2Traj, middleStack3Traj, middleBackboardStack3Traj, middleBackboardStack4Traj, middleEndTraj;

    public BlueClose2Plus4FarNew() {
        super(ScrappyConstants.AllianceType.BLUE, ScrappyConstants.AllianceSide.CLOSE, startingPose);
    }

    @Override
    public void initAuto() {
        // Middle
        middleSpikeMark = robot.m_drive.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startingPose), new Point(11.5, 34, Point.CARTESIAN))))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();

        middleBackboardTraj = robot.m_drive.pathBuilder()
                .addPath(new Path(new BezierLine(middleSpikeMark.getPath(0).getLastControlPoint(), new Point(52.5, 35, Point.CARTESIAN))))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.PI, 0.7)
                .build();

        middleStackTraj = robot.m_drive.pathBuilder()
                .addPath(new BezierLine(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(23.5, 57.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.2, () -> {
                    robot.m_outtake.back();
                })
                .addPath(new BezierLine(
                        new Point(23.5, 57.5, Point.CARTESIAN),
                        new Point(-16, 57.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(
                        new Point(-16, 57.5, Point.CARTESIAN),
                        new Point(-47, 35.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStackTraj = robot.m_drive.pathBuilder()
                .addPath(new BezierLine(
                        new Point(-55.25, 35.5, Point.CARTESIAN),
                        new Point(-50, 35.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.95, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .addPath(new BezierLine(
                        new Point(-50, 35.5, Point.CARTESIAN),
                        new Point(-40, 57.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStack2Traj = robot.m_drive.pathBuilder()
                .addPath(new BezierLine(
                        new Point(-40, 57.5, Point.CARTESIAN),
                        new Point(15, 57.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.9, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .addPath(new BezierCurve(
                        new Point(15, 57.5, Point.CARTESIAN),
                        new Point(18, 41, Point.CARTESIAN),
                        new Point(52, 41, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleStack2Traj = robot.m_drive.pathBuilder()
                .addPath(new BezierLine(
                        middleBackboardTraj.getPath(0).getLastControlPoint(),
                        new Point(23.5, 57.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.2, () -> {
                    robot.m_outtake.back();
                })
                .addPath(new BezierLine(
                        new Point(23.5, 57.5, Point.CARTESIAN),
                        new Point(-16, 57.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(
                        new Point(-16, 57.5, Point.CARTESIAN),
                        new Point(-47, 35.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStack3Traj = robot.m_drive.pathBuilder()
                .addPath(new BezierLine(
                        new Point(-55.25, 35.5, Point.CARTESIAN),
                        new Point(-50, 35.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.95, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .addPath(new BezierLine(
                        new Point(-50, 35.5, Point.CARTESIAN),
                        new Point(-40, 57.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStack4Traj = robot.m_drive.pathBuilder()
                .addPath(new BezierLine(
                        new Point(-40, 57.5, Point.CARTESIAN),
                        new Point(15, 57.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.9, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .addPath(new BezierCurve(
                        new Point(15, 57.5, Point.CARTESIAN),
                        new Point(18, 41, Point.CARTESIAN),
                        new Point(52, 41, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleEndTraj = robot.m_drive.pathBuilder()
                .addPath(new Path(new BezierLine(middleBackboardStack4Traj.getPath(1).getLastControlPoint(), new Point(middleBackboardStack4Traj.getPath(1).getLastControlPoint().getX()-4, middleBackboardStack2Traj.getPath(1).getLastControlPoint().getY() + 12, Point.CARTESIAN))))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.8, () -> {
                    robot.m_outtake.back();
                })
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
                            robot.m_intake.raise();
                            robot.m_outtake.extend();
                        }),
                        new FollowPath(robot.m_drive, middleBackboardTraj).alongWith(new ProfiledLiftCommand(robot.m_lift, 420)),
                        new WaitCommand(200),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(300),

                        new FollowPath(robot.m_drive, middleStackTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new WaitForReachedTValue(robot.m_drive, 0.1),
                                        new InstantCommand(robot.m_outtake::back),
                                        new ProfiledLiftCommand(robot.m_lift, 0),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new HoldPoint(robot.m_drive, new BezierPoint(middleStackTraj.getPath(2).getLastControlPoint()), Math.PI),
                        new InstantCommand(robot.m_intake::lower),
                        new WaitCommand(600),
                        new DriveToAprilTag(this, 9, false, new Pose2d(15.5)),
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
                        new FollowPath(robot.m_drive, middleBackboardStackTraj),
                        new HoldPoint(robot.m_drive, new BezierPoint(middleBackboardStackTraj.getPath(1).getLastControlPoint()), Math.PI),
                        new WaitCommand(100),
                        new FollowPath(robot.m_drive, middleBackboardStack2Traj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new WaitForReachedTValue(robot.m_drive, 0.7),
                                        new ProfiledLiftCommand(robot.m_lift, 900)
                                )
                        ),
                        new HoldPoint(robot.m_drive, new BezierPoint(middleBackboardStack2Traj.getPath(1).getLastControlPoint()), Math.PI),
                        new WaitCommand(300),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(300),

                        new FollowPath(robot.m_drive, middleStack2Traj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new WaitForReachedTValue(robot.m_drive, 0.1),
                                        new InstantCommand(robot.m_outtake::back),
                                        new ProfiledLiftCommand(robot.m_lift, 0),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new HoldPoint(robot.m_drive, new BezierPoint(middleStack2Traj.getPath(2).getLastControlPoint()), Math.PI),
                        new InstantCommand(robot.m_intake::lower),
                        new WaitCommand(600),
                        new DriveToAprilTag(this, 9, false, new Pose2d(15.5)),
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
                        new FollowPath(robot.m_drive, middleBackboardStack3Traj),
                        new HoldPoint(robot.m_drive, new BezierPoint(middleBackboardStack3Traj.getPath(1).getLastControlPoint()), Math.PI),
                        new WaitCommand(100),
                        new FollowPath(robot.m_drive, middleBackboardStack4Traj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new WaitForReachedTValue(robot.m_drive, 0.7),
                                        new ProfiledLiftCommand(robot.m_lift, 900)
                                )
                        ),
                        new HoldPoint(robot.m_drive, new BezierPoint(middleBackboardStack4Traj.getPath(1).getLastControlPoint()), Math.PI),
                        new WaitCommand(300),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(300),
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
