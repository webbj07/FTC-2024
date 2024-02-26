package org.firstinspires.ftc.team4100worlds.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team4100worlds.ScrappyCore;
import org.firstinspires.ftc.team4100worlds.ScrappySettings;
import org.firstinspires.ftc.team4100worlds.commands.InitPositions;
import org.firstinspires.ftc.team4100worlds.interfaces.PropDetector;
import org.firstinspires.ftc.team4100worlds.vision.PropDetectionProcessor;
import org.firstinspires.ftc.team4100worlds.vision.StackProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
public abstract class ScrappyAutoBase extends CommandOpMode {
    protected final ScrappySettings.AllianceType m_allianceType;
    protected final ScrappySettings.AllianceSide m_allianceSide;
    public PropDetector.DetectionResult detectionResult = PropDetector.DetectionResult.LEFT;
    public ScrappyCore robot;
    public WebcamName webcam1, webcam2;
    public Pose2d m_startPose;

    protected PropDetectionProcessor propDetectionProcessor;
    protected AprilTagProcessor aprilTagProcessor;
    public StackProcessor stackProcessor;
    public VisionPortal vision;

    public ScrappyAutoBase(ScrappySettings.AllianceType allianceType, ScrappySettings.AllianceSide allianceSide, Pose2d startPose) {
        m_allianceType = allianceType;
        m_allianceSide = allianceSide;
        m_startPose = startPose;
    }

    public ArrayList<AprilTagDetection> getAprilTagDetections(boolean isBackCamera) {
        if (!vision.getProcessorEnabled(aprilTagProcessor)) {
            vision.setProcessorEnabled(aprilTagProcessor, true);
        }

        return aprilTagProcessor.getDetections();
    }

    @Override
    public void initialize() {
        robot = new ScrappyCore(hardwareMap, m_allianceType, m_allianceSide);
        robot.m_drive.setStartingPose(m_startPose);
        new InitPositions(robot.m_lift, robot.m_outtake, robot.m_intake).schedule();

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        propDetectionProcessor = new PropDetectionProcessor(m_allianceType, m_allianceSide);
        stackProcessor = new StackProcessor();
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        vision = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessors(stackProcessor, propDetectionProcessor, aprilTagProcessor)
                .build();

        while (vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for cameras to be ready...");
            telemetry.update();
            sleep(10);
        }

        vision.setActiveCamera(webcam2);
        vision.setProcessorEnabled(stackProcessor, false);
        vision.setProcessorEnabled(aprilTagProcessor, false);
        vision.setProcessorEnabled(propDetectionProcessor, true);

        telemetry.addLine("Initializing trajectories...");
        telemetry.update();
        initAuto();

        while (opModeInInit()) {
            detectionResult = propDetectionProcessor.getDetectionResult();
            telemetry.addData("Detected", detectionResult);
            telemetry.update();
            sleep(25);
        }

        vision.setProcessorEnabled(propDetectionProcessor, false);

        if (m_allianceSide == ScrappySettings.AllianceSide.FAR) {
            vision.setProcessorEnabled(aprilTagProcessor, true);
        } else {
            vision.setProcessorEnabled(stackProcessor, true);
        }

        startAuto();
    }

    @Override
    public void run() {
        super.run();
        robot.m_drive.update();
    }

    public abstract void initAuto();
    public abstract void startAuto();
}
