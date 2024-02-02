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
    public static final double chassisWidth = 14;
    public static final double chassisHeight = 14;
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

        Action blueLeftAction = robot.getDrive().actionBuilder(new Pose2d(12, 61.75, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(22.8, 36))
                .strafeToLinearHeading(new Vector2d(40, 45), Math.toRadians(170))
                .strafeToLinearHeading(new Vector2d(60 - 9, 40), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(45, 34.5))
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .waitSeconds(0.001)
                .strafeToLinearHeading(new Vector2d(-61, 11.5), Math.toRadians(180))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(30, 11.5))
                .splineToSplineHeading(new Pose2d(35, 20, Math.toRadians(200)), Math.toRadians(90))
                .waitSeconds(0.001)
                .strafeToLinearHeading(new Vector2d(60 - 9, 28), Math.toRadians(180))
                .build();

        Action blueMiddleAction = robot.getDrive().actionBuilder(new Pose2d(12, 61.75, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(14, 33.51))
                .strafeToLinearHeading(new Vector2d(40, 38), Math.toRadians(170))
                .strafeToLinearHeading(new Vector2d(60 - 9, 35), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(50.5, 34.5))
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .waitSeconds(0.001)
                .strafeToLinearHeading(new Vector2d(-61, 11.5), Math.toRadians(180))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(30, 11.5))
                .splineToSplineHeading(new Pose2d(35, 20, Math.toRadians(200)), Math.toRadians(90))
                .waitSeconds(0.001)
                .strafeToLinearHeading(new Vector2d(60 - 9, 28), Math.toRadians(180))
                .build();

        Action blueRightAction = robot.getDrive().actionBuilder(new Pose2d(12, 61.75, Math.toRadians(90.00)))
                .strafeToLinearHeading(new Vector2d(5, 33.51), Math.toRadians(75))
                .strafeToLinearHeading(new Vector2d(40, 35), Math.toRadians(155))
                .strafeToLinearHeading(new Vector2d(60 - 9, 30), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(45, 25))
                .splineToConstantHeading(new Vector2d(30, 11.5), Math.PI)
                .splineToConstantHeading(new Vector2d(-30, 11.5), Math.PI)
                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(150)), Math.PI)
                .waitSeconds(0.001)
                .strafeToLinearHeading(new Vector2d(-61, 11.5), Math.toRadians(180))
                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(30, 11.5))
                .splineToSplineHeading(new Pose2d(35, 20, Math.toRadians(200)), Math.toRadians(90))
                .waitSeconds(0.001)
                .strafeToLinearHeading(new Vector2d(60 - 9, 28), Math.toRadians(180))
                .build();

        Action blueFar = robot.getDrive().actionBuilder(new Pose2d(-58.5, 36, Math.toRadians(180.00)))
                .strafeToLinearHeading(new Vector2d(-48, 45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12.5, 59.2), 0)
                .splineToConstantHeading(new Vector2d(40, 33), 0)
                .build();

        robot.runAction(blueFar);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}