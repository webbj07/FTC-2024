//package org.firstinspires.ftc.team4100worlds.autonomous;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.team4100worlds.ScrappyConstants;
//import org.firstinspires.ftc.team4100worlds.commands.DriveToAprilTag;
//import org.firstinspires.ftc.team4100worlds.commands.FollowPath;
//import org.firstinspires.ftc.team4100worlds.commands.HoldPoint;
//import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierCurve;
//import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierLine;
//import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.BezierPoint;
//import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.PathChain;
//import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;
//
//@Autonomous
//public class BlueFar2Plus5 extends ScrappyAutoBase {
//    public static Pose2d startingPose = new Pose2d(-38, 61.75, Math.toRadians(270));
//    private PathChain leftSpikeMark, leftStackTraj, leftBackboardTraj, leftBackboard2Traj, leftStackCycleTraj, leftBackboard3Traj, leftBackboard4Traj, leftBackboard5Traj, leftBackboard6Traj, leftStackCycle2Traj, endPath;
//
//    public BlueFar2Plus5() {
//        super(ScrappyConstants.AllianceType.BLUE, ScrappyConstants.AllianceSide.FAR, startingPose);
//    }
//
//    @Override
//    public void initAuto() {
//        // Middle
//        leftSpikeMark = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(new Point(startingPose), new Point(-39, 37, Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(330))
//                .build();
//
//        leftStackTraj = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(leftSpikeMark.getPath(0).getLastControlPoint(), new Point(-48.5, 35.5, Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(Math.toRadians(320), Math.PI, 0.6)
//                .build();
//
//        leftBackboardTraj = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(-55.25, 35.5, Point.CARTESIAN),
//                        new Point(-50, 35.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.95, () -> {
//                    robot.m_intake.raise();
//                    robot.m_intake.back();
//                })
//                .addPath(new BezierLine(
//                        new Point(-50, 35.5, Point.CARTESIAN),
//                        new Point(-40, 57.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//
//        leftBackboard2Traj = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(-40, 57.5, Point.CARTESIAN),
//                        new Point(15, 57.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.9, () -> {
//                    robot.m_outtake.extend(-0.07);
//                    robot.m_lift.setRelativePosition(500);
//                    robot.m_conveyor.stop();
//                    robot.m_intake.stop();
//                })
//                .addPath(new BezierCurve(
//                        new Point(15, 57.5, Point.CARTESIAN),
//                        new Point(18, 41, Point.CARTESIAN),
//                        new Point(52, 41, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//
//        leftBackboard3Traj = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(-55.25, 35.5, Point.CARTESIAN),
//                        new Point(-50, 35.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.95, () -> {
//                    robot.m_intake.raise();
//                    robot.m_intake.back();
//                })
//                .addPath(new BezierLine(
//                        new Point(-50, 35.5, Point.CARTESIAN),
//                        new Point(-40, 57.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//
//        leftBackboard4Traj = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(-40, 57.5, Point.CARTESIAN),
//                        new Point(15, 57.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.9, () -> {
//                    robot.m_outtake.extend(-0.07);
//                    robot.m_lift.setRelativePosition(800);
//                    robot.m_conveyor.stop();
//                    robot.m_intake.stop();
//                })
//                .addPath(new BezierCurve(
//                        new Point(15, 57.5, Point.CARTESIAN),
//                        new Point(18, 41, Point.CARTESIAN),
//                        new Point(52, 41, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//
//        leftBackboard5Traj = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(-50, 35.5, Point.CARTESIAN),
//                        new Point(-40, 57.5, Point.CARTESIAN)
//                ))
//                .addParametricCallback(0, () -> {
//                    robot.m_intake.raise();
//                    robot.m_intake.back();
//                })
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//
//        leftBackboard6Traj = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(-40, 57.5, Point.CARTESIAN),
//                        new Point(15, 57.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.9, () -> {
//                    robot.m_outtake.extend(-0.07);
//                    robot.m_lift.setRelativePosition(800);
//                    robot.m_conveyor.stop();
//                    robot.m_intake.stop();
//                })
//                .addPath(new BezierCurve(
//                        new Point(15, 57.5, Point.CARTESIAN),
//                        new Point(18, 41, Point.CARTESIAN),
//                        new Point(52, 41, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//
////        leftStackCycleTraj = robot.m_drive.pathBuilder()
////                .addPath(new BezierCurve(
////                        new Point(52, 40, Point.CARTESIAN),
////                        new Point(52, 60, Point.CARTESIAN),
////                        new Point(-47, 83, Point.CARTESIAN),
////                        new Point(-47, 35.5, Point.CARTESIAN)
////                ))
////                .setConstantHeadingInterpolation(Math.PI)
////                .addParametricCallback(0.1, () -> {
////                    robot.m_lift.toInitial();
////                    robot.m_outtake.back();
////                })
////                .build();
//
//        leftStackCycleTraj = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(52, 41, Point.CARTESIAN),
//                        new Point(23.5, 57.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.2, () -> {
//                    robot.m_lift.toInitial();
//                    robot.m_outtake.back();
//                })
//                .addPath(new BezierLine(
//                        new Point(23.5, 57.5, Point.CARTESIAN),
//                        new Point(-16, 57.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .addPath(new BezierLine(
//                        new Point(-16, 57.5, Point.CARTESIAN),
//                        new Point(-47, 35.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//
//        leftStackCycle2Traj = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(52, 41, Point.CARTESIAN),
//                        new Point(23.5, 57.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .addParametricCallback(0.2, () -> {
//                    robot.m_lift.toInitial();
//                    robot.m_outtake.back();
//                })
//                .addPath(new BezierLine(
//                        new Point(23.5, 57.5, Point.CARTESIAN),
//                        new Point(-16, 57.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .addPath(new BezierLine(
//                        new Point(-16, 57.5, Point.CARTESIAN),
//                        new Point(-47, 35.5, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//
//        endPath = robot.m_drive.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(52, 41, Point.CARTESIAN),
//                        new Point(47, 40, Point.CARTESIAN)
//                ))
//                .addParametricCallback(0.7, () -> {
//                    robot.m_lift.toInitial();
//                    robot.m_outtake.back();
//                })
//                .build();
//    }
//
//    @Override
//    public void startAuto() {
//        schedule(
//                new SequentialCommandGroup(
//                        new InstantCommand(robot.m_intake::lower),
//                        new FollowPath(robot.m_drive, leftSpikeMark),
//                        new SequentialCommandGroup(
//                                new InstantCommand(robot.m_intake::back),
//                                new InstantCommand(robot.m_intake::raise)
//                        ),
//                        new WaitCommand(300),
//                        new FollowPath(robot.m_drive, leftStackTraj),
//                        new HoldPoint(robot.m_drive, new BezierPoint(leftStackTraj.getPath(0).getLastControlPoint()), Math.PI),
//                        new WaitCommand(100),
//                        new InstantCommand(robot.m_intake::lower),
//                        new WaitCommand(600),
//                        new DriveToAprilTag(this, 9, false, new Pose2d(15.5)),
//                        new WaitCommand(200),
//                        new InstantCommand(() -> {
//                            robot.m_intake.suck();
//                            robot.m_conveyor.up();
//                            robot.m_intake.grab();
//                        }),
//                        new WaitCommand(300),
//                        new FollowPath(robot.m_drive, leftBackboardTraj),
//                        new HoldPoint(robot.m_drive, new BezierPoint(leftBackboardTraj.getPath(1).getLastControlPoint()), Math.PI),
//                        new WaitCommand(100),
//                        new FollowPath(robot.m_drive, leftBackboard2Traj),
//                        new HoldPoint(robot.m_drive, new BezierPoint(leftBackboard2Traj.getPath(1).getLastControlPoint()), Math.PI),
//                        new WaitCommand(300),
//                        new InstantCommand(robot.m_outtake::drop),
//                        new WaitCommand(300),
//                        new InstantCommand(() -> robot.m_lift.setRelativePosition(200)),
//                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(700)),
//
//                        new FollowPath(robot.m_drive, leftStackCycleTraj).alongWith(
//                                new SequentialCommandGroup(
//                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
//                                        new InstantCommand(robot.m_outtake::lower)
//                                )
//                        ),
//                        new HoldPoint(robot.m_drive, new BezierPoint(leftStackCycleTraj.getPath(2).getLastControlPoint()), Math.PI),
//                        new InstantCommand(robot.m_intake::lower),
//                        new WaitCommand(600),
//                        new DriveToAprilTag(this, 9, false, new Pose2d(15.5)),
//                        new InstantCommand(() -> {
//                            robot.m_intake.suck();
//                            robot.m_conveyor.up();
//                            robot.m_intake.grab();
//                        }),
//                        new WaitCommand(250),
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> robot.m_intake.backOne()),
//                                new WaitCommand(400),
//                                new InstantCommand(() -> robot.m_intake.backTwo()),
//                                new WaitCommand(400),
//                                new InstantCommand(() -> robot.m_intake.grab()),
//                                new WaitCommand(200)
//                        ),
//                        new FollowPath(robot.m_drive, leftBackboard3Traj),
//                        new HoldPoint(robot.m_drive, new BezierPoint(leftBackboard3Traj.getPath(1).getLastControlPoint()), Math.PI),
//                        new WaitCommand(100),
//                        new FollowPath(robot.m_drive, leftBackboard4Traj),
//                        new HoldPoint(robot.m_drive, new BezierPoint(leftBackboard4Traj.getPath(1).getLastControlPoint()), Math.PI),
//                        new WaitCommand(300),
//                        new InstantCommand(robot.m_outtake::drop),
//                        new WaitCommand(300),
//
//                        new FollowPath(robot.m_drive, leftStackCycle2Traj).alongWith(
//                                new SequentialCommandGroup(
//                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
//                                        new InstantCommand(robot.m_outtake::lower)
//                                )
//                        ),
//                        new HoldPoint(robot.m_drive, new BezierPoint(leftStackCycle2Traj.getPath(2).getLastControlPoint()), Math.PI),
//                        new WaitCommand(200),
//                        new InstantCommand(() -> {
//                            robot.m_intake.suck();
//                            robot.m_conveyor.up();
//                        }),
//                        new DriveToAprilTag(this, 9, false, new Pose2d(10)),
//                        new WaitCommand(600),
//                        new FollowPath(robot.m_drive, leftBackboard5Traj),
//                        new HoldPoint(robot.m_drive, new BezierPoint(leftBackboard5Traj.getPath(0).getLastControlPoint()), Math.PI),
//                        new WaitCommand(100),
//                        new FollowPath(robot.m_drive, leftBackboard6Traj),
//                        new HoldPoint(robot.m_drive, new BezierPoint(leftBackboard6Traj.getPath(1).getLastControlPoint()), Math.PI),
//                        new WaitCommand(300),
//                        new InstantCommand(robot.m_outtake::drop),
//                        new WaitCommand(300),
//                        new FollowPath(robot.m_drive, endPath).alongWith(
//                                new SequentialCommandGroup(
//                                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
//                                        new InstantCommand(robot.m_outtake::lower)
//                                )
//                        )
//                )
//        );
//    }
//}