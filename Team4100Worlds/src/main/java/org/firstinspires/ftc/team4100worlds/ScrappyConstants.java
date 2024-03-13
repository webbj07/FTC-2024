package org.firstinspires.ftc.team4100worlds;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class ScrappyConstants {
    /*
     * Game/season specific enums.
     */
    public enum AllianceType {
        BLUE, RED
    }
    public enum AllianceSide {
        CLOSE, FAR
    }

    /*
     * Used for debugging/tuning purposes and disallowing FTC Dashboard utilities during competitions.
     */
    public static boolean IS_COMPETITION = false;

    /*
     * Lengths and widths of various components.
     */
    public static final double CHASSIS_LENGTH = 17.4;
    public static final double CHASSIS_WIDTH = 15.8;
    public static final double INTAKE_EXTENSION_LENGTH = 5.4;

    /*
     * Orientation of the Expansion Hub, used for a non-orthogonal IMU setup.
     */
    public static RevHubOrientationOnRobot CONTROL_HUB_ORIENTATION = new RevHubOrientationOnRobot(xyzOrientation(180, 90, -21.9095));

    /*
     * These are the relative positions of each sensor from the center of the
     * robot (inches).
     */
    public static Vector2d FRONT_CAMERA_POSITION = new Vector2d(-0.5, 5.5);
    public static Vector2d BACK_CAMERA_POSITION = new Vector2d(-4, 7);
    public static Vector2d FRONT_DISTANCE_SENSOR_POSITION = new Vector2d(-4.25, 6.75);
    public static Vector2d LEFT_DISTANCE_SENSOR_POSITION = new Vector2d(-6, 4.5);
    public static Vector2d RIGHT_DISTANCE_SENSOR_POSITION = new Vector2d(6, 4.5);

    /*
     * Various positions for the different components.
     */
    public static class Positions {
        public static class Intake {
            public static double EXTENSION_UP = 0.59;
            public static double EXTENSION_DOWN = 0.22;
            public static double GRABBER_ONE_IDLE = 0.3;
            public static double GRABBER_TWO_IDLE = 0.48;
            public static double GRABBER_ONE_GRAB = 0.6;
            public static double GRABBER_TWO_GRAB = 0.13;
        }

        public static class Outtake {
            public final static double DROPPER_OUT = 0.225;
            public final static double DROPPER_IN = 0.38;
            public final static double EXTENSION_MIN = 0.517; // 0.52
            public final static double EXTENSION_MAX = 0.95;
        }

        public static class Lift {
            public final static int MAX = 1100;
        }

        public static class Plane {
            public final static double LAUNCH = 0.525;
            public final static double IDLE = 0.4;
        }
    }

    public static double DISTANCE_SENSOR_WIDTH = 9.3;
}
