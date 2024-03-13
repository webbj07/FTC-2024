package org.firstinspires.ftc.team4100worlds;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.MathFunctions;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Point;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Vector;
import org.firstinspires.ftc.team4100worlds.util.CustomPIDFCoefficients;

@Config
public class FollowerConstants {

    // This section is for setting the actual drive vector for the front left wheel
    private static double xMovement = 70;
    private static double yMovement = -53.6;
    private static double[] convertToPolar = Point.cartesianToPolar(xMovement, yMovement);
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0],convertToPolar[1]));

    // Mass of robot in kilograms
    public static double mass = 20.41;

    // Large heading error PIDF coefficients
    public static CustomPIDFCoefficients largeHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            2,
            0,
            0.003,
            0);

    public static double largeHeadingPIDFFeedForward = 0.015;

    public static double headingPIDFSwitch = Math.PI/20;

    // Small heading error PIDF coefficients
    public static CustomPIDFCoefficients smallHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            2.3,
            0,
            0.1,
            0);

    public static double smallHeadingPIDFFeedForward = 0.015;

    // Small translational PIDF coefficients
    public static CustomPIDFCoefficients smallTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.05,
            0,
            0,
            0);

    // Small translational Integral value
    public static CustomPIDFCoefficients smallTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    public static double smallTranslationalPIDFFeedForward = 0.015;

    public static double translationalPIDFSwitch = 3;

    // Large translational PIDF coefficients
    public static CustomPIDFCoefficients largeTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.1,
            0,
            0.005,
            0);

    public static double largeTranslationalPIDFFeedForward = 0.01;

    // Large translational Integral
    public static CustomPIDFCoefficients largeTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    // Small drive PIDF coefficients
    public static CustomPIDFCoefficients smallDrivePIDFCoefficients = new CustomPIDFCoefficients(
            0.04,
            0,
            0,
            0);

    public static double smallDrivePIDFFeedForward = 0.01;

    public static double drivePIDFSwitch = 10;

    // Large drive PIDF coefficients
    public static CustomPIDFCoefficients largeDrivePIDFCoefficients = new CustomPIDFCoefficients(
            0.04,
            0,
            0,
            0);

    public static double largeDrivePIDFFeedForward = 0.01;

    // Centrifugal force to power scaling
    public static double centrifugalScaling = 0.001;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    // this is for curves
    public static double forwardZeroPowerAcceleration = -23.5;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    // this is for curves
    public static double lateralZeroPowerAcceleration = -61.5;

    // A multiplier for the zero power acceleration to increase the speed of the robot.
    // increasing this will cause the robot to drive faster, at the risk of end of path overshoots or localization slippage
    // decreasing this will cause the deceleration at the end of the path to be less, making the robot slower but reducing risk of end of path overshoots or localization slippage
    public static double zeroPowerAccelerationMultiplier = 3;

    // When the drivetrain is at the end of its current path or path chain and the velocity goes
    // below this value, then end the path. This is in inches/second
    // This can be custom set for each path
    public static double pathEndVelocity = 0.1;

    // When the drivetrain is at the end of its current path or path chain and the translational error goes
    // below this value, then end the path. This is in inches
    // This can be custom set for each path
    public static double pathEndTranslational = 0.1;

    // When the drivetrain is at the end of its current path or path chain and the heading error goes
    // below this value, then end the path. This is in radians
    // This can be custom set for each path
    public static double pathEndHeading = 0.007;

    // When the t-value of the closest point to the robot on the path is greater than this value,
    // then the path is considered at its end.
    // This can be custom set for each path
    public static double pathEndTValue = 0.99;

    // When the path is considered at its end parametrically, then the follower has this many
    // seconds to further correct by default.
    // This can be custom set for each path
    public static double pathEndTimeout = 1.5;
}