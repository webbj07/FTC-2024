package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ScrappySettings;
import org.firstinspires.ftc.teamcode.subsystem.SensorLocalization;

@Autonomous
public class REV2mRealignmentTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        SensorLocalization sensors = new SensorLocalization(hardwareMap);

        double angle = 0;

        while (!isStarted()) {
            double dL = sensors.getLeftDistance();
            double dR = sensors.getRightDistance();

            angle = Math.atan(Math.abs((dR - dL)) / ScrappySettings.DISTANCE_SENSOR_WIDTH);

            telemetry.addLine("Left distance (inches): " + dL);
            telemetry.addLine("Right distance (inches): " + dR);
            telemetry.addLine("Angle (degrees): " + Math.toDegrees(angle));
            telemetry.update();
        }

        waitForStart();

        Action turn = drive.actionBuilder(new Pose2d(0, 0, 0))
                .turnTo(angle)
                .build();

        Actions.runBlocking(turn);
    }
}