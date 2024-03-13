package org.firstinspires.ftc.team4100worlds.autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.team4100worlds.pedropathing.follower.Follower;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Vector;
import org.firstinspires.ftc.team4100worlds.subsystem.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous
public class AprilTagHeadingCorrection extends LinearOpMode {
    private Follower follower;
    private Intake intake;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal vision;
    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap);
        follower.setAuto(false);

        intake = new Intake(hardwareMap);
        intake.raise();

        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        vision = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(aprilTagProcessor)
                .build();

        while (vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for cameras to be ready...");
            telemetry.update();
            sleep(10);
        }

        ExposureControl exposureControl = vision.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long)6, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = vision.getCameraControl(GainControl.class);
        gainControl.setGain(250);
        sleep(20);

        while (opModeInInit()) {
            telemetry.addLine("Ready!");
            telemetry.update();
            sleep(10);
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            if (detections.size() > 0) {
                double yawError = detections.get(0).ftcPose.yaw;
                double angle = Math.toRadians(-yawError) + follower.getPose().getHeading();

                telemetry.addData("yawError (deg)", yawError);
                telemetry.addData("yawError (rad)", Math.toRadians(yawError));

                Vector headingCorrection = follower.getHeadingVectorAprilTag(angle);
                follower.setMovementVectors(new Vector(), headingCorrection, new Vector());
            }

            follower.update();
            telemetry.update();
        }
    }
}
