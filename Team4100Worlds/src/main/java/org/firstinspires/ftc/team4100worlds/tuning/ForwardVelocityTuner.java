package org.firstinspires.ftc.team4100worlds.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team4100worlds.pedropathing.localization.PoseUpdater;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.MathFunctions;
import org.firstinspires.ftc.team4100worlds.pedropathing.pathgeneration.Vector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ForwardVelocityTuner extends OpMode {
    private ArrayList<Double> velocities = new ArrayList<>();
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> motors;

    private PoseUpdater poseUpdater;

    public static double DISTANCE = 40, RECORD_NUMBER = 10;

    private Telemetry telemetryA;

    private boolean end;

    @Override
    public void init() {
        poseUpdater = new PoseUpdater(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftRear = hardwareMap.get(DcMotorEx.class, "LB");
        rightRear = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add((double) 0);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
        telemetryA.addLine("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetryA.addLine("Press cross or A to stop");
        telemetryA.update();

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        leftFront.setPower(1);
        leftRear.setPower(1);
        rightFront.setPower(1);
        rightRear.setPower(1);
        end = false;
    }

    @Override
    public void loop() {
        if (gamepad1.cross || gamepad1.a) {
            requestOpModeStop();
        }

        poseUpdater.update();
        if (!end) {
            if (Math.abs(poseUpdater.getPose().getX()) > DISTANCE) {
                end = true;
                for (DcMotorEx motor : motors) {
                    motor.setPower(0);
                }
            } else {
                double currentVelocity = Math.abs(MathFunctions.dotProduct(poseUpdater.getVelocity(), new Vector(1, 0)));
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            double average = 0;
            for (Double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();

            telemetryA.addData("forward velocity:", average);
            telemetryA.update();
        }
    }

    @Override
    public void stop() {
        super.stop();
    }
}