package org.firstinspires.ftc.team4100worlds.autonomous.blue.close;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.team4100worlds.ScrappyConstants;
import org.firstinspires.ftc.team4100worlds.autonomous.ScrappyAutoBase;
import org.firstinspires.ftc.team4100worlds.commands.DriveToAprilTag;
import org.firstinspires.ftc.team4100worlds.commands.FollowPath;
import org.firstinspires.ftc.team4100worlds.commands.LocalizeWithBackboard;
import org.firstinspires.ftc.team4100worlds.commands.ProfiledLiftCommand;
import org.firstinspires.ftc.team4100worlds.commands.WaitForPartner;
import org.firstinspires.ftc.team4100worlds.commands.WaitForReachedTValue;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierCurve;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;

public class BlueClose2Plus2TrussSafe extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    private PathChain leftSpikeMarkPath, middleSpikeMarkPath, rightSpikeMarkPath,
        backboardPath, backboardWaitPathOne, backboardCyclePathOne, stackCyclePathOne, endTraj;

    public BlueClose2Plus2TrussSafe() {
        super(ScrappyConstants.AllianceType.BLUE, ScrappyConstants.AllianceSide.CLOSE, startingPose);
    }

    @Override
    public void initAuto() {
        // Left
        leftSpikeMarkPath = robot.m_drive.pathBuilder()
            .addPath(new BezierLine(
                new Point(startingPose),
                new Point(32, 38, Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(225))
            .build();

        // Middle
        middleSpikeMarkPath = robot.m_drive.pathBuilder()
            .addPath(new BezierLine(
                new Point(startingPose),
                new Point(17, 35.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(startingPose.getHeading())
            .build();

        // Right
        rightSpikeMarkPath = robot.m_drive.pathBuilder()
            .addPath(new BezierLine(
                new Point(startingPose),
                new Point(10.2, 38.5, Point.CARTESIAN))
            )
            .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(220))
            .build();
    }

    @Override
    public void startAuto() {
        PathChain spikeMarkPath;
        double backboardInitialY, backboardCycleY;

        switch (detectionResult) {
            case LEFT:
                spikeMarkPath = leftSpikeMarkPath;
                backboardInitialY = 42.5;
                backboardCycleY = 35;
                break;
            case MIDDLE:
                spikeMarkPath = middleSpikeMarkPath;
                backboardInitialY = 35;
                backboardCycleY = 42.5;
                break;
            default:
                spikeMarkPath = rightSpikeMarkPath;
                backboardInitialY = 27;
                backboardCycleY = 35;
                break;
        }

        backboardPath = robot.m_drive.pathBuilder()
            .addPath(new BezierLine(
                spikeMarkPath.getPath(0).getLastControlPoint(),
                new Point(52, backboardInitialY, Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(spikeMarkPath.getPath(spikeMarkPath.size() - 1).getHeadingGoal(1), Math.PI, 0.5)
            .addParametricCallback(0.5, () -> robot.m_drive.setMaxPower(0.3))
            .build();

        stackCyclePathOne = robot.m_drive.pathBuilder()
            .addPath(new BezierLine(
                backboardPath.getPath(0).getLastControlPoint(),
                new Point(35, 52)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(35, 52),
                new Point(2, 59, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(2, 59, Point.CARTESIAN),
                new Point(-16, 59, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(-16, 59, Point.CARTESIAN),
                new Point(-47, 35.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addParametricCallback(0, () -> robot.m_drive.setMaxPower(0.8))
            .build();

        backboardWaitPathOne = robot.m_drive.pathBuilder()
            .addPath(new BezierLine(
                new Point(-55.25, 35.5, Point.CARTESIAN),
                new Point(-50, 35.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addParametricCallback(0.9, () -> {
                robot.m_intake.raise();
                robot.m_intake.back();
            })
            .addPath(new BezierLine(
                new Point(-50, 35.5, Point.CARTESIAN),
                new Point(-40, 57, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(-40, 57, Point.CARTESIAN),
                new Point(62, 60, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .build();

        backboardCyclePathOne = robot.m_drive.pathBuilder()
            .addPath(new BezierCurve(
                new Point(62, 60, Point.CARTESIAN),
                new Point(33, 57.5, Point.CARTESIAN),
                new Point(43.5, backboardCycleY, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addParametricCallback(0.35, () -> {
                robot.m_lift.setPosition(800);
                robot.m_outtake.extend();
            })
            .build();

        endTraj = robot.m_drive.pathBuilder()
            .addPath(new Path(new BezierLine(backboardCyclePathOne.getPath(0).getLastControlPoint(), new Point(47, 60, Point.CARTESIAN))))
            .setConstantHeadingInterpolation(Math.PI)
            .build();

        schedule(
            new SequentialCommandGroup(
                new InstantCommand(robot.m_intake::lower),
                new FollowPath(robot.m_drive, spikeMarkPath),
                new InstantCommand(robot.m_intake::back),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    robot.m_intake.raise();
                    robot.m_outtake.extend();
                }).raceWith(new ProfiledLiftCommand(robot.m_lift, 435)),
                new WaitCommand(200),
                new InstantCommand(robot.m_intake::raise),
                new InstantCommand(() -> robot.m_drive.setMaxPower(0.5)),
                new FollowPath(robot.m_drive, backboardPath),
                new WaitCommand(300),
                new InstantCommand(robot.m_outtake::drop),
                new WaitCommand(350),
                new ProfiledLiftCommand(robot.m_lift, 535),
                new WaitCommand(200),
                new ProfiledLiftCommand(robot.m_lift, 635),
                new WaitCommand(100),
                new InstantCommand(() -> robot.m_drive.setMaxPower(1)),

                new FollowPath(robot.m_drive, stackCyclePathOne).alongWith(
                    new SequentialCommandGroup(
                        new WaitCommand(200),
                        new WaitForReachedTValue(robot.m_drive, 0.1),
                        new InstantCommand(robot.m_outtake::back),
                        new ProfiledLiftCommand(robot.m_lift, 0),
                        new InstantCommand(robot.m_outtake::lower)
                    )
                ),
                new InstantCommand(robot.m_intake::lower),
                new WaitCommand(350),
                new DriveToAprilTag(this, 9, false, new Pose2d(14.75)),
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
                new FollowPath(robot.m_drive, backboardWaitPathOne),
                new WaitForPartner(robot.m_sensors, true),
                new InstantCommand(() -> {
                    robot.m_intake.stop();
                    robot.m_conveyor.stop();
                    robot.m_intake.raise();
                }),
                new FollowPath(robot.m_drive, backboardCyclePathOne),
                new LocalizeWithBackboard(this, 1.5, true, true),
                new WaitCommand(750),
                new InstantCommand(() -> robot.m_drive.setMaxPower(1)),
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
