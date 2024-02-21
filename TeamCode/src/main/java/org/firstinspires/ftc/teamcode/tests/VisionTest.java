package org.firstinspires.ftc.teamcode.tests;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.vision.StackProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
        Intake intake = new Intake(hardwareMap);
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        WebcamName webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        StackProcessor stackProcessor = new StackProcessor();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(stackProcessor)
                .build();

        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for cameras to be ready...");
            telemetry.update();
            sleep(10);
        }

        portal.setActiveCamera(webcam2);
        portal.setProcessorEnabled(stackProcessor, true);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        Vector2d error = null;
        while (error == null) {
            error = stackProcessor.getDistanceError(drive.pose.heading.toDouble());
        }

        telemetry.addData("x", error.x);
        telemetry.addData("y", error.y);
        telemetry.update();

        Action driveTo = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .stopAndAdd(intake::lower)
                .strafeToLinearHeading(new Vector2d(error.x + 12, error.y + 1.5), Math.toRadians(180))
                .build();

        Actions.runBlocking(driveTo);

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}
