package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public final class ParallelSlidesVelTuner extends LinearOpMode {
    public static double RUNTIME = 2;
    private DcMotorEx leftSlide, rightSlide;
    private double maxVel = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        leftSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "RSlide");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        timer.reset();

        while (opModeIsActive()) {
            if (timer.seconds() < RUNTIME) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);

                double velLeft = leftSlide.getVelocity();
                double velRight = rightSlide.getVelocity();
                double avgVel = (velLeft + velRight) / 2.0;

                if (avgVel > maxVel) {
                    maxVel = avgVel;
                }
            } else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }

            telemetry.addData("maxVel", maxVel);
            telemetry.update();
        }
    }
}