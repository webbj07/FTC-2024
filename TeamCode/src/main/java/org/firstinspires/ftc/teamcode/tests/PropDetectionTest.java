package org.firstinspires.ftc.teamcode.tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ScrappyConstants;
import org.firstinspires.ftc.teamcode.vision.PropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class PropDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        WebcamName webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        PropDetectionProcessor propProcessor = new PropDetectionProcessor(ScrappyConstants.AllianceType.RED, ScrappyConstants.AllianceSide.FAR);

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(propProcessor)
                .build();

        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for cameras to be ready...");
            telemetry.update();
            sleep(10);
        }

        portal.setActiveCamera(webcam2);
        portal.setProcessorEnabled(propProcessor, true);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            telemetry.addData("Detected", propProcessor.getDetectionResult());
            telemetry.update();
        }
    }
}
