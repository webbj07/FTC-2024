package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public final class SplineTestEx extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-39, 61.75, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-35.23, 33.51))
                        .strafeToLinearHeading(new Vector2d(-54, 36.4), Math.toRadians(180.00))
//                .waitSeconds(0.25)
                        .strafeToLinearHeading(new Vector2d(-58.5, 36), Math.toRadians(180.00))
//                .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-50, 45), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(12.5, 58.5), 0)
                        .splineToConstantHeading(new Vector2d(40, 38), 0)
                        .strafeToLinearHeading(new Vector2d(60 - 9, 35), Math.toRadians(180))
                        .waitSeconds(0.1)
                        .splineToConstantHeading(new Vector2d(46, 59.5), 0)
                        .build());
    }
}
