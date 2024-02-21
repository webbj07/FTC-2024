package org.firstinspires.ftc.team4100worlds;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class ScrappySettings {
    public enum AllianceType {
        BLUE, RED
    }
    public enum AllianceSide {
        CLOSE, FAR
    }

    public static boolean IS_COMPETITION = false;
    public static double FRONT_CAMERA_OFFSET_X = -0.5;
    public static double FRONT_CAMERA_OFFSET_Y = 5.5;
    public static double BACK_CAMERA_OFFSET_X = -4;
    public static double BACK_CAMERA_OFFSET_Y = 7;
    public static double DISTANCE_SENSOR_WIDTH = 9.3;
    public static RevHubOrientationOnRobot CONTROL_HUB_ORIENTATION = new RevHubOrientationOnRobot(xyzOrientation(180, 90, -21.9095));
}
