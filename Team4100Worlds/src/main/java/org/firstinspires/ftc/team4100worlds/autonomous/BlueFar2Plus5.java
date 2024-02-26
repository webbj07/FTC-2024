package org.firstinspires.ftc.team4100worlds.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team4100worlds.ScrappySettings;
import org.firstinspires.ftc.team4100worlds.commands.FollowPath;
import org.firstinspires.ftc.team4100worlds.commands.HoldPoint;
import org.firstinspires.ftc.team4100worlds.commands.LocalizeWithStack;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierCurve;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierPoint;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Path;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;
import org.firstinspires.ftc.team4100worlds.util.Timer;

@Autonomous
public class BlueFar2Plus5 extends ScrappyAutoBase {
    public static Pose2d startingPose = new Pose2d(-38, 61.75, Math.toRadians(270));
    private PathChain leftSpikeMark, leftStackTraj, leftBackboardTraj, leftBackboard2Traj, leftStackCycleTraj;

    public BlueFar2Plus5() {
        super(ScrappySettings.AllianceType.BLUE, ScrappySettings.AllianceSide.FAR, startingPose);
    }

    @Override
    public void initAuto() {
        // Middle
        leftSpikeMark = robot.m_drive.pathBuilder()
                .addPath(new BezierLine(new Point(startingPose), new Point(-35, 37, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(330))
                .build();

        leftStackTraj = robot.m_drive.pathBuilder()
                .addPath(new BezierLine(leftSpikeMark.getPath(0).getLastControlPoint(), new Point(-47, 35.5, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(330), Math.PI)
                .build();

        leftBackboardTraj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-55.25, 35.5, Point.CARTESIAN),
                        new Point(-50, 35.5, Point.CARTESIAN),
                        new Point(-40, 60, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.08, () -> {
                    robot.m_intake.raise();
                    robot.m_intake.back();
                })
                .build();

        leftBackboard2Traj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(-40, 60, Point.CARTESIAN),
                        new Point(26.4, 61.5, Point.CARTESIAN),
                        new Point(51.5, 41, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.5, () -> {
                    robot.m_outtake.extend(-0.07);
                    robot.m_lift.setRelativePosition(600);
                    robot.m_conveyor.stop();
                    robot.m_intake.stop();
                })
                .build();

        leftStackCycleTraj = robot.m_drive.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(51.5, 41, Point.CARTESIAN),
                        new Point(36, 61, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.05, () -> {
                    robot.m_lift.toInitial();
                    robot.m_outtake.back();
                })
                .addPath(new BezierCurve(
                        new Point(0, 61, Point.CARTESIAN),
                        new Point(-40, 61, Point.CARTESIAN),
                        new Point(-47, 35.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .build();
    }

    @Override
    public void startAuto() {
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(robot.m_intake::lower),
                        new FollowPath(robot.m_drive, leftSpikeMark),
                        new SequentialCommandGroup(
                                new InstantCommand(robot.m_intake::back),
                                new WaitCommand(250),
                                new InstantCommand(robot.m_intake::raise)
                        ),
                        new FollowPath(robot.m_drive, leftStackTraj),
                        new HoldPoint(robot.m_drive, new BezierPoint(leftStackTraj.getPath(0).getLastControlPoint()), Math.PI),
                        new WaitCommand(500),
                        new InstantCommand(robot.m_intake::lower),
                        new WaitCommand(400),
                        new DriveToAprilTag(this, 9, false, new Pose2d(15)),
                        new InstantCommand(() -> {
                            robot.m_intake.suck();
                            robot.m_conveyor.up();
                            robot.m_intake.grab();
                        }),
                        new WaitCommand(300),
                        new InstantCommand(() -> vision.setActiveCamera(webcam1)),
                        new FollowPath(robot.m_drive, leftBackboardTraj),
                        new HoldPoint(robot.m_drive, new BezierPoint(leftBackboardTraj.getPath(0).getLastControlPoint()), Math.PI),
                        new WaitCommand(100),
                        new FollowPath(robot.m_drive, leftBackboard2Traj),
                        new InstantCommand(robot.m_outtake::drop),
                        new WaitCommand(300),
                        new FollowPath(robot.m_drive, leftStackCycleTraj).alongWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                                        new InstantCommand(robot.m_outtake::lower)
                                )
                        ),
                        new InstantCommand(robot.m_intake::lower)

                )
        );
    }
}
