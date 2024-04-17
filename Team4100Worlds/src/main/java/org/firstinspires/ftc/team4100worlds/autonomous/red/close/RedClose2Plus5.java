package org.firstinspires.ftc.team4100worlds.autonomous.red.close;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.team4100worlds.ScrappyConstants;
import org.firstinspires.ftc.team4100worlds.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.team4100worlds.commands.FollowPath;
import org.firstinspires.ftc.team4100worlds.commands.GotoStack;
import org.firstinspires.ftc.team4100worlds.commands.HoldPoint;
import org.firstinspires.ftc.team4100worlds.commands.LocalizeWithBackboard;
import org.firstinspires.ftc.team4100worlds.commands.LocalizeWithStack;
import org.firstinspires.ftc.team4100worlds.commands.ProfiledLiftCommand;
import org.firstinspires.ftc.team4100worlds.commands.WaitForReachedTValue;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierCurve;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierPoint;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;

public class RedClose2Plus5 extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(15.5, -61.75, Math.toRadians(90));
    private PathChain leftSpikeMark, middleSpikeMark, rightSpikeMark,
        leftBackboardTraj, middleBackboardTraj, rightBackboardTraj,
        leftStackTraj, middleStackTraj, rightStackTraj,
        stackTraj, stackTraj2,
        backboardTraj, backboardTraj2, backboardTraj3,
        endTraj;

    public RedClose2Plus5() {
        super(ScrappyConstants.AllianceType.RED, ScrappyConstants.AllianceSide.CLOSE, startingPose);
    }

    @Override
    public void initAuto() {
        // Right
        rightSpikeMark = robot.m_drive.pathBuilder()
            .addPath(new Path(new BezierLine(new Point(startingPose), new Point(10.2, -38.5, Point.CARTESIAN))))
            .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(140))
            .build();

        rightBackboardTraj = robot.m_drive.pathBuilder()
            .addPath(new Path(new BezierLine(rightSpikeMark.getPath(0).getLastControlPoint(), new Point(51.5, -28.3, Point.CARTESIAN))))
            .setLinearHeadingInterpolation(Math.toRadians(140), Math.PI, 0.7)
            .build();

        rightStackTraj = robot.m_drive.pathBuilder()
            .addPath(new BezierCurve(
                rightBackboardTraj.getPath(0).getLastControlPoint(),
                new Point(30, -10.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(30, -10.5, Point.CARTESIAN),
                new Point(-45, -11.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .build();

        // Middle
        middleSpikeMark = robot.m_drive.pathBuilder()
            .addPath(new Path(new BezierLine(new Point(startingPose), new Point(17, -35.5, Point.CARTESIAN))))
            .setConstantHeadingInterpolation(startingPose.getHeading())
            .build();

        middleBackboardTraj = robot.m_drive.pathBuilder()
            .addPath(new Path(new BezierLine(middleSpikeMark.getPath(0).getLastControlPoint(), new Point(51.5, -35, Point.CARTESIAN))))
            .setLinearHeadingInterpolation(startingPose.getHeading(), Math.PI, 0.7)
            .build();

        middleStackTraj = robot.m_drive.pathBuilder()
            .addPath(new BezierCurve(
                middleBackboardTraj.getPath(0).getLastControlPoint(),
                new Point(30, -10.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(30, -10.5, Point.CARTESIAN),
                new Point(-45, -11.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .build();

        // Left
        leftSpikeMark = robot.m_drive.pathBuilder()
            .addPath(new Path(new BezierLine(new Point(startingPose), new Point(32, -38, Point.CARTESIAN))))
            .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(135))
            .build();

        leftBackboardTraj = robot.m_drive.pathBuilder()
            .addPath(new Path(new BezierLine(leftSpikeMark.getPath(0).getLastControlPoint(), new Point(51.5, -42.5, Point.CARTESIAN))))
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.PI, 0.7)
            .build();

        leftStackTraj = robot.m_drive.pathBuilder()
            .addPath(new BezierCurve(
                leftBackboardTraj.getPath(0).getLastControlPoint(),
                new Point(30, -10.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(30, -10.5, Point.CARTESIAN),
                new Point(-45, -11.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .build();

        backboardTraj = robot.m_drive.pathBuilder()
            .addPath(new BezierCurve(
                new Point(-40, -10.5, Point.CARTESIAN),
                new Point(37, -10.5, Point.CARTESIAN),
                new Point(48, -30, Point.CARTESIAN)
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
            .addParametricCallback(0.8, () -> robot.m_drive.setMaxPower(0.3))
            .build();

        stackTraj = robot.m_drive.pathBuilder()
            .addPath(new BezierCurve(
                backboardTraj.getPath(0).getLastControlPoint(),
                new Point(30, -10.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(30, -10.5, Point.CARTESIAN),
                new Point(-45, -11.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .build();

        backboardTraj2 = robot.m_drive.pathBuilder()
            .addPath(new BezierCurve(
                new Point(-40, -10.5, Point.CARTESIAN),
                new Point(37, -10.5, Point.CARTESIAN),
                new Point(48, -30, Point.CARTESIAN)
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
            .addParametricCallback(0.8, () -> robot.m_drive.setMaxPower(0.3))
            .build();
        stackTraj2 = robot.m_drive.pathBuilder()
            .addPath(new BezierCurve(
                backboardTraj2.getPath(0).getLastControlPoint(),
                new Point(30, -10.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(30, -10.5, Point.CARTESIAN),
                new Point(-65, -11.75, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addParametricCallback(0.8, () -> {
                robot.m_intake.suck();
                robot.m_conveyor.up();
            })
            .build();

        backboardTraj3 = robot.m_drive.pathBuilder()
            .addPath(new BezierCurve(
                new Point(-40, -10.5, Point.CARTESIAN),
                new Point(37, -10.5, Point.CARTESIAN),
                new Point(48, -30, Point.CARTESIAN)
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
            .addParametricCallback(0.8, () -> robot.m_drive.setMaxPower(0.3))
            .build();
        endTraj = robot.m_drive.pathBuilder()
            .addPath(new Path(new BezierLine(backboardTraj3.getPath(0).getLastControlPoint(), new Point(47, -29, Point.CARTESIAN))))
            .setConstantHeadingInterpolation(Math.PI)
            .build();
    }

    @Override
    public void startAuto() {
        PathChain spikeMarkPath, initialBackboardPath, stackPath;

        switch (detectionResult) {
            case LEFT:
                spikeMarkPath = rightSpikeMark;
                initialBackboardPath = rightBackboardTraj;
                stackPath = rightStackTraj;
                break;
            case MIDDLE:
                spikeMarkPath = middleSpikeMark;
                initialBackboardPath = middleBackboardTraj;
                stackPath = middleStackTraj;
                break;
            default:
                spikeMarkPath = leftSpikeMark;
                initialBackboardPath = leftBackboardTraj;
                stackPath = leftStackTraj;
                break;
        }

        vision.setProcessorEnabled(aprilTagProcessor, false);
        vision.setProcessorEnabled(stackProcessor, true);

        schedule(
            new SequentialCommandGroup(
                new InstantCommand(robot.m_intake::lower),
                new FollowPath(robot.m_drive, spikeMarkPath),
                new InstantCommand(() -> {
                    robot.m_intake.back();
                    robot.m_outtake.extend();
                }).raceWith(new ProfiledLiftCommand(robot.m_lift, 435)),
                new WaitCommand(200),
                new InstantCommand(robot.m_intake::raise),
                new InstantCommand(() -> robot.m_drive.setMaxPower(0.5)),
                new FollowPath(robot.m_drive, initialBackboardPath),
                new WaitCommand(300),
                new InstantCommand(robot.m_outtake::drop),
                new WaitCommand(350),
                new ProfiledLiftCommand(robot.m_lift, 535),
                new WaitCommand(200),
                new ProfiledLiftCommand(robot.m_lift, 635),
                new WaitCommand(100),
                new InstantCommand(() -> robot.m_drive.setMaxPower(1)),

                new FollowPath(robot.m_drive, stackPath).alongWith(
                    new SequentialCommandGroup(
                        new WaitCommand(200),
                        new WaitForReachedTValue(robot.m_drive, 0.05),
                        new InstantCommand(robot.m_outtake::back),
                        new ProfiledLiftCommand(robot.m_lift, 0),
                        new InstantCommand(robot.m_outtake::lower)
                    )
                ),
                new LocalizeWithStack(this),
                new HoldPoint(robot.m_drive, new BezierPoint(stackPath.getPath(1).getLastControlPoint()), Math.PI),
                new InstantCommand(robot.m_intake::lower),
                new WaitCommand(300),
                new GotoStack(this),
                new InstantCommand(() -> {
                    robot.m_intake.suck();
                    robot.m_conveyor.up();
                    robot.m_intake.grab();
                }),
                new WaitCommand(150),
                new SequentialCommandGroup(
                    new InstantCommand(() -> robot.m_intake.backOne()),
                    new WaitCommand(150),
                    new InstantCommand(() -> robot.m_intake.backTwo()),
                    new WaitCommand(150),
                    new InstantCommand(() -> robot.m_intake.grab()),
                    new WaitCommand(150)
                ),
                new FollowPath(robot.m_drive, backboardTraj).alongWith(
                    new SequentialCommandGroup(
                        new WaitCommand(300),
                        new WaitForReachedTValue(robot.m_drive, 0.5),
                        new ProfiledLiftCommand(robot.m_lift, 800)
                    )
                ),
                new HoldPoint(robot.m_drive, new BezierPoint(backboardTraj.getPath(0).getLastControlPoint()), Math.PI),
                new WaitCommand(500),
                new LocalizeWithBackboard(this, 1.8),
                new WaitCommand(500),
                new InstantCommand(robot.m_outtake::drop),
                new WaitCommand(350),

                new FollowPath(robot.m_drive, stackTraj).alongWith(
                    new SequentialCommandGroup(
                        new WaitCommand(200),
                        new WaitForReachedTValue(robot.m_drive, 0.05),
                        new InstantCommand(robot.m_outtake::back),
                        new ProfiledLiftCommand(robot.m_lift, 0),
                        new InstantCommand(robot.m_outtake::lower)
                    )
                ),
                new LocalizeWithStack(this),
                new HoldPoint(robot.m_drive, new BezierPoint(stackTraj.getPath(1).getLastControlPoint()), Math.PI),
                new InstantCommand(robot.m_intake::lower),
                new WaitCommand(300),
                new GotoStack(this),
                new InstantCommand(() -> {
                    robot.m_intake.suck();
                    robot.m_conveyor.up();
                    robot.m_intake.grab();
                }),
                new WaitCommand(150),
                new SequentialCommandGroup(
                    new InstantCommand(() -> robot.m_intake.backOne()),
                    new WaitCommand(150),
                    new InstantCommand(() -> robot.m_intake.backTwo()),
                    new WaitCommand(150),
                    new InstantCommand(() -> robot.m_intake.grab()),
                    new WaitCommand(150)
                ),
                new FollowPath(robot.m_drive, backboardTraj2).alongWith(
                    new SequentialCommandGroup(
                        new WaitCommand(300),
                        new WaitForReachedTValue(robot.m_drive, 0.5),
                        new ProfiledLiftCommand(robot.m_lift, 900)
                    )
                ),
                new HoldPoint(robot.m_drive, new BezierPoint(backboardTraj2.getPath(0).getLastControlPoint()), Math.PI),
                new WaitCommand(500),
                new LocalizeWithBackboard(this, 1.5),
                new WaitCommand(500),
                new InstantCommand(robot.m_outtake::drop),
                new WaitCommand(350),

                new FollowPath(robot.m_drive, stackTraj2).alongWith(
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
                new FollowPath(robot.m_drive, backboardTraj3).alongWith(
                    new SequentialCommandGroup(
                        new WaitCommand(300),
                        new WaitForReachedTValue(robot.m_drive, 0.5),
                        new ProfiledLiftCommand(robot.m_lift, 1000)
                    )
                ),
                new HoldPoint(robot.m_drive, new BezierPoint(backboardTraj3.getPath(0).getLastControlPoint()), Math.PI),
                new WaitCommand(500),
                new LocalizeWithBackboard(this, 1.5),
                new WaitCommand(500),
                new InstantCommand(robot.m_outtake::drop),
                new WaitCommand(350),

                new FollowPath(robot.m_drive, endTraj).alongWith(
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
