package org.firstinspires.ftc.team4100worlds;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class FieldConstants {
    public static final double PixelWidth = 3.25; // in
    public static final double TileLength = 24; // in
    public static final double MaxWallCoordinate = 71;
    public static class SpikeMark {
        public static final Vector2d BlueCloseLeft = new Vector2d(23, 30);
        public static final Vector2d BlueCloseMiddle = new Vector2d(11.5, 24);
        public static final Vector2d BlueCloseRight = new Vector2d(0.75, 30);

        public static final Vector2d BlueFarLeft = new Vector2d(-24, 30);
        public static final Vector2d BlueFarMiddle = new Vector2d(-35, 24);
        public static final Vector2d BlueFarRight = new Vector2d(-46.25, 30);

        public static final Vector2d RedCloseLeft = new Vector2d(0.75, -30);
        public static final Vector2d RedCloseMiddle = new Vector2d(11.5, -24);
        public static final Vector2d RedCloseRight = new Vector2d(23, -30);

        public static final Vector2d RedFarLeft = new Vector2d(-46.25, -30);
        public static final Vector2d RedFarMiddle = new Vector2d(-35, -24);
        public static final Vector2d RedFarRight = new Vector2d(-24, -30);
    }
    public static class PixelStack {
        public static final Vector2d BlueLeft = new Vector2d(-69, 11.5);
        public static final Vector2d BlueMiddle = new Vector2d(-69, 23.5);
        public static final Vector2d BlueRight = new Vector2d(-69, 35.5);

        public static final Vector2d RedLeft = new Vector2d(-69, -35.5);
        public static final Vector2d RedMiddle = new Vector2d(-69, -23.5);
        public static final Vector2d RedRight = new Vector2d(-69, -11.5);
    }
    public static class Backboard {
        public static final Vector2d BlueLeft = new Vector2d(59.25, 41.5);
        public static final Vector2d BlueMiddle = new Vector2d(59.25, 35.25);
        public static final Vector2d BlueRight = new Vector2d(59.25, 29);

        public static final Vector2d RedLeft = new Vector2d(59.25, -29);
        public static final Vector2d RedMiddle = new Vector2d(59.25, -35.25);
        public static final Vector2d RedRight = new Vector2d(59.25, -41.5);
    }
}
