package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.ScrappyCore;
import org.firstinspires.ftc.teamcode.ScrappyConstants;
import org.firstinspires.ftc.teamcode.commands.InitPositions;
import org.firstinspires.ftc.teamcode.interfaces.PropDetector;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.vision.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.vision.StackProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
public abstract class ScrappyAutoBase extends CommandOpMode {
    protected final ScrappyConstants.AllianceType m_allianceType;
    protected final ScrappyConstants.AllianceSide m_allianceSide;
    protected final Pose2d m_startingPose;
    public PropDetector.DetectionResult detectionResult = PropDetector.DetectionResult.LEFT;
    public ScrappyCore robot;
    public WebcamName webcam1, webcam2;

    protected PropDetectionProcessor propDetectionProcessor;
    protected AprilTagProcessor aprilTagProcessor;
    public StackProcessor stackProcessor;
    public VisionPortal vision;

    public ScrappyAutoBase(ScrappyConstants.AllianceType allianceType, ScrappyConstants.AllianceSide allianceSide, Pose2d startPose) {
        m_allianceType = allianceType;
        m_allianceSide = allianceSide;
        m_startingPose = startPose;
    }

    public void localizeWithRawAprilTag(boolean isBackCamera, AprilTagDetection detection) {
        Pose2d robotPos = AprilTagLocalization.getRobotPositionFromTag(detection, robot.m_drive.pose.heading.toDouble(), isBackCamera);
        this.robot.m_drive.pose = robotPos;
    }

    public ArrayList<AprilTagDetection> getAprilTagDetections(boolean isBackCamera) {
        if (!vision.getProcessorEnabled(aprilTagProcessor)) {
            vision.setProcessorEnabled(aprilTagProcessor, true);
        }

        return aprilTagProcessor.getDetections();
    }

    @Override
    public void initialize() {
        robot = new ScrappyCore(hardwareMap, m_allianceType, m_allianceSide, m_startingPose);
        new InitPositions(robot.m_lift, robot.m_outtake, robot.m_intake).schedule();

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        propDetectionProcessor = new PropDetectionProcessor(m_allianceType, m_allianceSide);
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        vision = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessors(aprilTagProcessor, propDetectionProcessor)
                .build();

        while (vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for cameras to be ready...");
            telemetry.update();
            sleep(10);
        }

        vision.setActiveCamera(webcam2);
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

        vision.setProcessorEnabled(aprilTagProcessor, true);
        vision.setProcessorEnabled(propDetectionProcessor, false);

        startAuto();
    }

    @Override
    public void run() {
        super.run();
        robot.m_drive.updatePoseEstimate();
        PoseStorage.currentPose = robot.m_drive.pose;
    }

    public abstract void initAuto();
    public abstract void startAuto();
}
