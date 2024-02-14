package com.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    enum Detection {
        LEFT, MIDDLE, RIGHT
    }
    public static final double chassisWidth = 15;
    public static final double chassisHeight = 17.5;
    public static final double maxVel = 50;
    public static final double maxAccel = 50;
    public static final double maxAngVel = Math.PI;
    public static final double maxAngAccel = Math.PI;
    public static final double trackWidth = 12.9;
    public static final Detection detectedSide = Detection.LEFT;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
                .setConstraints(maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth)
                .setDimensions(chassisWidth, chassisHeight)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();
//
        Action blueLeftAction = robot.getDrive().actionBuilder(new Pose2d(15.5, 61.75, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(30, 37, Math.toRadians(230)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(51.5, 40.75), Math.toRadians(180))

//                .strafeTo(new Vector2d(11.5, 36.5))
//                .strafeToLinearHeading(new Vector2d(51.5, 35), Math.PI)

                .splineToLinearHeading(new Pose2d(10, 37, Math.toRadians(210)), Math.PI)
                .strafeToLinearHeading(new Vector2d(51.5, 28.5), Math.PI)
                .endTrajectory()

                // l 18.89
                // m 19.39
                // r 19.61

                .strafeToConstantHeading(new Vector2d(45, 34.5))
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .endTrajectory()

                .strafeToLinearHeading(new Vector2d(-61, 11.5), Math.toRadians(180))
                .endTrajectory()

                .strafeToConstantHeading(new Vector2d(30, 11.5))
                .splineToConstantHeading(new Vector2d(52, 30), 0)
                .endTrajectory()

                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .endTrajectory()

                .strafeToLinearHeading(new Vector2d(-61, 11.5), Math.toRadians(180))
                .endTrajectory()

                .strafeToConstantHeading(new Vector2d(30, 11.5))
                .splineToConstantHeading(new Vector2d(49, 29.5), 0)
                .endTrajectory()
                .build();
//
//        Action blueMiddleAction = robot.getDrive().actionBuilder(new Pose2d(12, 61.75, Math.toRadians(90.00)))
//                .strafeTo(new Vector2d(14, 33.51))
//                .strafeToLinearHeading(new Vector2d(40, 38), Math.toRadians(170))
//                .strafeToLinearHeading(new Vector2d(60 - 9, 35), Math.toRadians(180))
//                .strafeToConstantHeading(new Vector2d(50.5, 34.5))
//                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
//                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
//                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
//                .waitSeconds(0.001)
//                .strafeToLinearHeading(new Vector2d(-61, 11.5), Math.toRadians(180))
//                .waitSeconds(0.001)
//                .strafeToConstantHeading(new Vector2d(30, 11.5))
//                .splineToSplineHeading(new Pose2d(35, 20, Math.toRadians(200)), Math.toRadians(90))
//                .waitSeconds(0.001)
//                .strafeToLinearHeading(new Vector2d(60 - 9, 28), Math.toRadians(180))
//                .build();
//
//        Action blueRightAction = robot.getDrive().actionBuilder(new Pose2d(12, 61.75, Math.toRadians(90.00)))
//                .strafeToLinearHeading(new Vector2d(5, 33.51), Math.toRadians(75))
//                .strafeToLinearHeading(new Vector2d(40, 35), Math.toRadians(155))
//                .strafeToLinearHeading(new Vector2d(60 - 9, 30), Math.toRadians(180))
//                .strafeToConstantHeading(new Vector2d(45, 25))
//                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
//                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
//                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
//                .waitSeconds(0.001)
//                .strafeToLinearHeading(new Vector2d(-61, 11.5), Math.toRadians(180))
//                .waitSeconds(0.001)
//                .strafeToConstantHeading(new Vector2d(30, 11.5))
//                .splineToSplineHeading(new Pose2d(35, 20, Math.toRadians(200)), Math.toRadians(90))
//                .waitSeconds(0.001)
//                .strafeToLinearHeading(new Vector2d(60 - 9, 28), Math.toRadians(180))
//                .build();
//
//        Action blueFar = robot.getDrive().actionBuilder(new Pose2d(-58.5, 35.5, Math.toRadians(180)))
////                .strafeToLinearHeading(new Vector2d(-55, 40.5), Math.toRadians(180))
////                .splineToConstantHeading(new Vector2d(12.5, 59.5), 0)
////                .splineToConstantHeading(new Vector2d(36, 33.5), 0)
////                .waitSeconds(0.001)
////                .strafeToConstantHeading(new Vector2d(49, 36))
////                .waitSeconds(0.001)
////                .strafeToLinearHeading(new Vector2d(47, 38), Math.toRadians(180.00))
////                .splineToConstantHeading(new Vector2d(-36, 58), Math.toRadians(190))
////                .splineToConstantHeading(new Vector2d(-52, 35.5), Math.toRadians(180))
//                .lineToXConstantHeading(-50)
//                .splineToConstantHeading(new Vector2d(-34, 59), 0)
//                .splineToConstantHeading(new Vector2d(5, 59), 0)
//                .splineToSplineHeading(new Pose2d(25, 53.1, Math.toRadians(150)), 0)
//                .splineToConstantHeading(new Vector2d(26.4, 53), 0)
//                .splineToLinearHeading(new Pose2d(49, 36, Math.toRadians(180)), 0)
//                .waitSeconds(0.001)
//                .splineToConstantHeading(new Vector2d(40, 55), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-35, 55), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-52, 33), Math.toRadians(180))
////                .lineToXConstantHeading(35.5) 4.27 3.28
////                .splineToConstantHeading(new Vector2d(22.5, 58), Math.toRadians(180))
////                .splineToConstantHeading(new Vector2d(-35, 58), Math.toRadians(180))
////                .splineToConstantHeading(new Vector2d(-52, 35.5), Math.toRadians(180))
//                .build();

        Action blueFar2 = robot.getDrive().actionBuilder(new Pose2d(-39, 61.75, Math.toRadians(270)))
                // place on spike mark
                .splineToLinearHeading(new Pose2d(-36.5, 37, Math.toRadians(330)), 0)
                .waitSeconds(0.9)
                // goto stack without correction
                .strafeToLinearHeading(new Vector2d(-51, 35.5), Math.toRadians(180))
                .waitSeconds(0.35)
                .endTrajectory()
                // correct with april tag
                .strafeToLinearHeading(new Vector2d(-55, 35.5), Math.toRadians(180.00))
                .endTrajectory()
                // goto backboard
                .lineToXConstantHeading(-50)
                .splineToConstantHeading(new Vector2d(-36, 59), 0)
                .endTrajectory()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(5, 59), 0)
                .splineToConstantHeading(new Vector2d(26.4, 59), 0)
                .splineToConstantHeading(new Vector2d(49, 29.5), 0)
                .waitSeconds(0.1)
                .endTrajectory()
                // go back to stack
                .splineToConstantHeading(new Vector2d(27, 59), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-35, 59), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-51, 35.5), Math.toRadians(180))
                .endTrajectory()
                // correct with april tag
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-55, 35.5), Math.toRadians(180.00))
                .waitSeconds(1.05)
                .endTrajectory()
                // goto backboard
                .lineToXConstantHeading(-50)
                .splineToConstantHeading(new Vector2d(-36, 59), 0)
                .splineToConstantHeading(new Vector2d(5, 59), 0)
                .splineToConstantHeading(new Vector2d(26.4, 59), 0)
                .splineToConstantHeading(new Vector2d(49, 29.5), 0)
                .waitSeconds(0.2)
                .endTrajectory()
                // park
                .lineToX(43)
                .endTrajectory()
                .build();

        Action actualLeftClose = robot.getDrive().actionBuilder(new Pose2d(15.5, 61.75, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(30, 37, Math.toRadians(230)), Math.toRadians(270))
                .waitSeconds(0.4)
                .waitSeconds(0.1)
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(52.5, 40.25), Math.toRadians(180))
                .waitSeconds(0.1)
                .endTrajectory()

                .afterDisp(5, () -> {
                })
                .strafeToConstantHeading(new Vector2d(45, 34.5))
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .waitSeconds(0.5)
                .endTrajectory()

                .stopAndAdd(() -> {})
                .strafeToLinearHeading(new Vector2d(-55.5, 12.5), Math.toRadians(180))
                .waitSeconds(0.25)
                .waitSeconds(0.8)
                .endTrajectory()

                .afterDisp(5, () -> {
                })
                .strafeToConstantHeading(new Vector2d(30, 11))
                .afterDisp(15, () -> {
                })
                .splineToConstantHeading(new Vector2d(52.5, 31), 0)
                .waitSeconds(0.3)
                .endTrajectory()
                .build();

        Action underTruss = robot.getDrive().actionBuilder(new Pose2d(-39, 61.75, Math.toRadians(270)))
//                .strafeToLinearHeading(new Vector2d(-42, 14), Math.toRadians(90))
//                .strafeTo(new Vector2d(-40, 11.5))
//                .turnTo(Math.PI)
//                .strafeToLinearHeading(new Vector2d(-36.5, 13), Math.toRadians(90) - 1e-6)
//                .strafeToConstantHeading(new Vector2d(-40, 11.5))
//                .turnTo(Math.PI)
                .strafeTo(new Vector2d(-55, 16))
                .splineToLinearHeading(new Pose2d(-36.5, 13, Math.toRadians(90) - 1e-6), 0)
                .build();

        Action left2 = robot.getDrive().actionBuilder(new Pose2d(15.5, 61.75, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(30, 37, Math.toRadians(230)), Math.toRadians(270))
                .waitSeconds(0.4)
                .waitSeconds(0.1)
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(52.5, 40.25), Math.toRadians(180))
                .waitSeconds(0.1)
                .endTrajectory()
                .strafeToLinearHeading(new Vector2d(52.75, 33.5), Math.toRadians(180))
                .endTrajectory()
                .afterDisp(5, () -> {
                })
                .splineToConstantHeading(new Vector2d(50, 60), 0)
                .splineToConstantHeading(new Vector2d(59.5, 60), 0)
                .build();

        robot.runAction(underTruss);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}