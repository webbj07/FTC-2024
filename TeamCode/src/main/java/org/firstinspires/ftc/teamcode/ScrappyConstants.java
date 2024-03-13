package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class ScrappyConstants {
    public enum AllianceType {
        BLUE, RED
    }
    public enum AllianceSide {
        CLOSE, FAR
    }

    public static boolean IS_COMPETITION = false;

    /*
     * These are the relative positions of each component from the center of the
     * robot (inches).
     */
    public static Vector2d FRONT_CAMERA_POSITION = new Vector2d(-0.5, 5.5);
    public static Vector2d BACK_CAMERA_POSITION = new Vector2d(-4, 7);
    public static double FRONT_CAMERA_OFFSET_X = -0.5;
    public static double FRONT_CAMERA_OFFSET_Y = 5.5;
    public static double BACK_CAMERA_OFFSET_X = -4;
    public static double BACK_CAMERA_OFFSET_Y = 7;
    public static double DISTANCE_SENSOR_WIDTH = 9.3;
    public static RevHubOrientationOnRobot CONTROL_HUB_ORIENTATION = new RevHubOrientationOnRobot(xyzOrientation(180, 90, -21.9095));
}
