package org.firstinspires.ftc.team4100worlds.tuning;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HeadingPIDTuner extends LinearOpMode {
    public static double targetHeading = 0;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    private DcMotorEx LF, LB, RF, RB;
    private IMU imu;
    private PIDController controller;

    @Override
    public void runOpMode() {
        // Motors
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Controller
        controller = new PIDController(kP, kI, kD);
        controller.setSetPoint(Math.toRadians(targetHeading));

        telemetry.addData("Tuning", "Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controller.setPID(kP, kI, kD);

            double heading = getCorrectHeading();
            double power = controller.calculate(heading, Math.toRadians(targetHeading));

            LF.setPower(-power);
            LB.setPower(-power);
            RF.setPower(power);
            RB.setPower(power);

            telemetry.addData("Power", power);
            telemetry.addData("Current Heading", Math.toDegrees(heading));
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Error", Math.toDegrees(controller.getPositionError()));
            telemetry.update();
        }
    }

    private double getCorrectHeading() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        while (heading < 0) {
            heading += 2 * Math.PI;
        }

        heading %= (2 * Math.PI);

        return heading;
    }
}
