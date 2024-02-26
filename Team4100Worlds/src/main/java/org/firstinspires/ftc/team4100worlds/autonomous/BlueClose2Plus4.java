package org.firstinspires.ftc.team4100worlds.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team4100worlds.ScrappySettings;
import org.firstinspires.ftc.team4100worlds.commands.FollowPath;
import org.firstinspires.ftc.team4100worlds.commands.LocalizeWithStack;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierCurve;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;

@Autonomous
public class BlueClose2Plus4 extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    private PathChain middleSpikeMark, middleBackboardTraj, middleStackTraj, middleBackboardStackTraj, middleStack2Traj, middleBackboardStack2Traj, middleStack3Traj, middleBackboardStack3Traj, middleEndTraj;

    private DistanceSensor dist;
    public BlueClose2Plus4() {
        super(ScrappySettings.AllianceType.BLUE, ScrappySettings.AllianceSide.CLOSE, startingPose);
    }

    @Override
    public void initAuto() {
        dist = hardwareMap.get(DistanceSensor.class, "FrontDistance");

        // Middle
        middleSpikeMark = robot.m_drive.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startingPose), new Point(11.5, 35.5, Point.CARTESIAN))))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();

        middleBackboardTraj = robot.m_drive.pathBuilder()
                .addPath(new Path(new BezierLine(middleSpikeMark.getPath(0).getLastControlPoint(), new Point(52, 35, Point.CARTESIAN))))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.PI, 0.8)
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
                        new Point(-45, 11.75, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStackTraj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(51, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.1, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .addParametricCallback(0.5, () -> {
                    robot.m_intake.stop();
                    robot.m_conveyor.stop();
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(600);
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
                        new Point(-45, 11.75, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        middleBackboardStack2Traj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(51, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.1, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .addParametricCallback(0.5, () -> {
                    robot.m_intake.stop();
                    robot.m_conveyor.stop();
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(900);
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
                        new Point(51, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.1, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .addParametricCallback(0.5, () -> {
                    robot.m_intake.stop();
                    robot.m_conveyor.stop();
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(900);
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
                            robot.m_intake.raise();
                            robot.m_outtake.extend(-0.07);
                            robot.m_lift.setRelativePosition(420);
                        }),
                        new FollowPath(robot.m_drive, middleBackboardTraj),
                        new WaitCommand(200),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(300),
                        new FollowPath(robot.m_drive, middleStackTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(robot.m_lift::toInitial),
                                        new InstantCommand(robot.m_outtake::back),
                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new InstantCommand(robot.m_intake::lower),
                        new InstantCommand(() -> {
                            telemetry.addData("d", dist.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }),
                        new LocalizeWithStack(this),
                        new InstantCommand(() -> {
                            vision.setProcessorEnabled(stackProcessor, false);
                        }),
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
                                new WaitCommand(700)
                        ),
                        new FollowPath(robot.m_drive, middleBackboardStackTraj),
                        new WaitCommand(200),
                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(600)),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(300),

                        new FollowPath(robot.m_drive, middleStack2Traj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(robot.m_lift::toInitial),
                                        new InstantCommand(robot.m_outtake::back),
                                        new InstantCommand(() -> vision.setProcessorEnabled(stackProcessor, true)),
                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new InstantCommand(robot.m_intake::lower),
                        new LocalizeWithStack(this),
                        new InstantCommand(() -> {
                            vision.setProcessorEnabled(stackProcessor, false);
                        }),
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
                                new WaitCommand(700)
                        ),
                        new FollowPath(robot.m_drive, middleBackboardStack2Traj),
                        new WaitCommand(200),
                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(900)),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(300),

                        new FollowPath(robot.m_drive, middleStack3Traj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(robot.m_lift::toInitial),
                                        new InstantCommand(robot.m_outtake::back),
                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new InstantCommand(() -> {
                            robot.m_intake.suck();
                            robot.m_conveyor.up();
                        }),
                        new WaitCommand(700),
                        new FollowPath(robot.m_drive, middleBackboardStack3Traj),
                        new WaitCommand(200),
                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(900)),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(300),

                        new FollowPath(robot.m_drive, middleEndTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new InstantCommand(robot.m_lift::toInitial),
                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        )
                )
        );
    }
}
