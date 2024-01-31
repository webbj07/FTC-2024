package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalization;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public final class SelfDriveTest extends LinearOpMode {
    private final Pose2d startPose = new Pose2d(17.00, 61.75, Math.toRadians(90.00));
    private MecanumDrive drive;
    private Lift lift;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);
        lift = new Lift(hardwareMap);
        initAprilTag();

        telemetry.addData(">", "Touch play to start getting data");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            Action init = drive.actionBuilder(startPose)
                    .strafeToLinearHeading(new Vector2d(19.53, 21.64), Math.toRadians(210))
                    .build();
            Actions.runBlocking(init);

            while (true) {
                drive.updatePoseEstimate();
                if (telemetryAprilTag()) {
                    telemetry.update();
                    break;
                }
                telemetry.update();
            }

            visionPortal.close();

            Action goToTag = drive.actionBuilder(drive.pose)
                    .afterDisp(0, () -> lift.setRelativePosition(1200))
                    .strafeToLinearHeading(new Vector2d(60 - 15, 41.5), Math.toRadians(180))
                    .build();
            Actions.runBlocking(goToTag);

            sleep(15000);
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    private boolean telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.size() > 0) {
            AprilTagDetection detection = currentDetections.get(0);
            Pose2d currentPos = drive.pose;
            Pose2d estimatedPos = AprilTagLocalization.getRobotPositionFromTag(detection, currentPos.heading.toDouble(), true);

            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("%6.1f %6.1f", detection.ftcPose.x, detection.ftcPose.y));
            telemetry.addLine("Robot");
            telemetry.addLine(String.format("%6.1f %6.1f %6.1f", currentPos.position.x, currentPos.position.y, currentPos.heading.toDouble()));
            telemetry.addLine("Robot (From Tag)");
            telemetry.addLine(String.format("%6.1f %6.1f %6.1f", estimatedPos.position.x, estimatedPos.position.y, currentPos.heading.toDouble()));

            drive.pose = estimatedPos;
            return true;
        }

        return false;
    }
}