package org.firstinspires.ftc.teamcode.tuning;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalization;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraOffsetTuner extends LinearOpMode {
    public static double X_AWAY = 11;
    public static double Y_AWAY = 0;
    public static double OFFSET_X = 0;
    public static double OFFSET_Y = 0;
    public static int DESIRED_ID = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessors(aprilTagProcessor)
                .build();

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        double lastOffsetX = OFFSET_X;
        double lastOffsetY = OFFSET_Y;
        double lastXAway = X_AWAY;
        double lastYAway = Y_AWAY;

        while (opModeIsActive()) {
            AprilTagDetection goodDetection = null;
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id != DESIRED_ID) {
                    continue;
                }

                goodDetection = detection;
                break;
            }

            if (goodDetection == null) {
                telemetry.addLine("Cannot find tag...");
                telemetry.update();
                continue;
            }

            if (OFFSET_X != lastOffsetX || OFFSET_Y != lastOffsetY || Y_AWAY != lastYAway || X_AWAY != lastXAway) {
                lastOffsetX = OFFSET_X;
                lastOffsetY = OFFSET_Y;
                lastXAway = X_AWAY;
                lastYAway = Y_AWAY;

                drive.pose = AprilTagLocalization.getRobotPositionFromTag(goodDetection, drive.pose.heading.toDouble(), OFFSET_X, OFFSET_Y);
                Action runWithNewOffset = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(AprilTagLocalization.getTagPosition(DESIRED_ID).plus(new Vector2d(X_AWAY, Y_AWAY)), Math.toRadians(180))
                        .build();
                telemetry.addLine("Driving to tag...");
                telemetry.update();
                Actions.runBlocking(runWithNewOffset);
            }

            telemetry.addLine("Waiting for new offset...");
            telemetry.update();
        }
    }
}
