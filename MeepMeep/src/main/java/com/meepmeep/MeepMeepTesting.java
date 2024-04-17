package com.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.pedropathing.DefaultBotBuilder;
import com.noahbres.meepmeep.pedropathing.entity.PedroPathingBotEntity;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.BezierCurve;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.BezierLine;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.Path;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.PathChain;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.Point;

public class MeepMeepTesting {
    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    public static PathChain middleSpikeMark, m2, m3;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        PedroPathingBotEntity robot = new DefaultBotBuilder(meepMeep)
            // Set this to the length and width of your robot
            .setDimensions(15, 17.5)
            // Set this based on your follower constants for PedroPathing
            // (xMovement, yMovement, forwardZeroPowerAcceleration, lateralZeroPowerAcceleration, zeroPowerAccelerationMultiplier)
            .setConstraints(70, -50, -25, -60, 3)
            .build();

        middleSpikeMark = robot.getDrive().pathBuilder()
            .addPath(new Path(new BezierLine(new Point(startingPose), new Point(12, 37.5, Point.CARTESIAN))))
            .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(220))
            .addPath(new Path(new BezierLine(new Point(12, 37.5, Point.CARTESIAN), new Point(52.5, 27, Point.CARTESIAN))))
            .setLinearHeadingInterpolation(Math.toRadians(220), Math.PI, 0.7)
            .addPath(new BezierLine(
                new Point(52.5, 27, Point.CARTESIAN),
                new Point(30, 10.5, Point.CARTESIAN)
            ))
            .addParametricCallback(0.5, () -> {
            })
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(30, 10.5, Point.CARTESIAN),
                new Point(-65, 11.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierCurve(
                new Point(-65, 11.5, Point.CARTESIAN),
                new Point(-40, 10.5, Point.CARTESIAN),
                new Point(37, 10.5, Point.CARTESIAN),
                new Point(52.2, 28, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(52.2, 28, Point.CARTESIAN),
                new Point(30, 10.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(30, 10.5, Point.CARTESIAN),
                new Point(-65, 11.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierCurve(
                new Point(-65, 11.5, Point.CARTESIAN),
                new Point(-40, 10.5, Point.CARTESIAN),
                new Point(37, 10.5, Point.CARTESIAN),
                new Point(52.2, 28, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(52.2, 28, Point.CARTESIAN),
                new Point(30, 10.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(30, 10.5, Point.CARTESIAN),
                new Point(-65, 11.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addParametricCallback(0.8, () -> {
            })
            .addPath(new BezierCurve(
                new Point(-65, 11.5, Point.CARTESIAN),
                new Point(-40, 10.5, Point.CARTESIAN),
                new Point(37, 10.5, Point.CARTESIAN),
                new Point(52.2, 28, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new Path(new BezierLine(new Point(52.2, 28, Point.CARTESIAN), new Point(47, 35, Point.CARTESIAN))))
            .setConstantHeadingInterpolation(Math.PI)
            .build();

        double length = 17.4 / 2 + 5.4; // + 5.4
        double fflength = length / Math.sqrt(2);
        Vector2d BlueCloseRight = new Vector2d(0.75, 30);

        Path scoreSpikeMark = new Path(new BezierCurve(
            new Point(startingPose),
            new Point(28, 52, Point.CARTESIAN),
            new Point(0.75 + fflength, 30 + fflength, Point.CARTESIAN)
        ));

        m2 = robot.getDrive().pathBuilder()
            .addPath(scoreSpikeMark)
            .setLinearHeadingInterpolation(startingPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta())
            .addPath(new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(52.5, 27, Point.CARTESIAN))))
            .setLinearHeadingInterpolation(scoreSpikeMark.getEndTangent().getTheta(), Math.PI, 0.7)
            .build();

        m3 = robot.getDrive().pathBuilder()
            .addPath(new BezierLine(
                new Point(-55.25, 35.5, Point.CARTESIAN),
                new Point(-50, 35.5, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(-50, 35.5, Point.CARTESIAN),
                new Point(-40, 57, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(-40, 57, Point.CARTESIAN),
                new Point(23, 59, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .addPath(new BezierLine(
                new Point(23, 59, Point.CARTESIAN),
                new Point(49, 39, Point.CARTESIAN)
            ))
            .setConstantHeadingInterpolation(Math.PI)
            .build();

        robot.followPath(m3);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(robot)
            .start();
    }
}