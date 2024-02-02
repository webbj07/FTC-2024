package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public final class ServoTest extends LinearOpMode {
    public static String servoName = "IntakeEx";
    public static final double SERVO_POSITION_CHANGE = 0.05;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo servo = hardwareMap.get(Servo.class, servoName);

        waitForStart();

        boolean dlReleased = false, drReleased = false;

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                if (!dlReleased) {
                    servo.setPosition(servo.getPosition() - SERVO_POSITION_CHANGE);
                    dlReleased = true;
                }
            } else {
                dlReleased = false;
            }

            if (gamepad1.dpad_right) {
                if (!drReleased) {
                    servo.setPosition(servo.getPosition() + SERVO_POSITION_CHANGE);
                    drReleased = true;
                }
            } else {
                drReleased = false;
            }

            telemetry.addData("pos", servo.getPosition());
            telemetry.update();
        }
    }
}
