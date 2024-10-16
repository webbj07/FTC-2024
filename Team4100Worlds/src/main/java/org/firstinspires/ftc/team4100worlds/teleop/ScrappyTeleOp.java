package org.firstinspires.ftc.team4100worlds.teleop;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ScrappyTeleOp extends ScrappyTeleOpBase {
    private final ElapsedTime elapsedTime = new ElapsedTime();
    private final PIDController headingController = new PIDController(0, 0, 0);
    private double m_headingOffset = Math.PI;
    private GamepadEx gamepad1Ex, gamepad2Ex;
    private int m_slideTopPos = 0;
    private double m_speed = 1;
    private boolean isFieldCentric = false;
    private boolean isHeadingLock = false;
    private double lfPower = 0, lrPower = 0, rfPower = 0, rbPower = 0, denominator = 0;

    public ScrappyTeleOp() {
        super(null, null);
    }

    @Override
    public void initTeleOp() {
        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
        //
        //              Gamepad 1
        //
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(robot.intake::toggleGrabber)
        );
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(robot.intake::toggleRotation)
        );
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(robot.extendo::toggleExtended)
        );
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(robot.arm::toggleGrab)
        );
        //
        //              Gamepad 2
        //
//        gamepad2Ex.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new InstantCommand(robot.lift::run)
//        );
//        gamepad2Ex.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new InstantCommand(robot.lift::stop)
//        );

    }

    @Override
    public void run() {
        super.run();

        double lx = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double ly = -gamepad1.left_stick_y;

        lx *= m_speed;
        rx *= m_speed;
        ly *= m_speed;

        if (isFieldCentric) {
            this.driveFieldCentric(gamepad1, m_speed);
        } else {
            if (!isHeadingLock) {
                denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);
                lfPower = (ly + lx + rx) / denominator;
                lrPower = (ly - lx + rx) / denominator;
                rfPower = (ly - lx - rx) / denominator;
                rbPower = (ly + lx - rx) / denominator;
            } else {
                double heading = getCorrectHeading();
                double output = headingController.calculate(heading);

                denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(output), 1);

                lfPower = (ly + lx + output) / denominator;
                lrPower = (ly - lx + output) / denominator;
                rfPower = (ly - lx - output) / denominator;
                rbPower = (ly + lx - output) / denominator;
            }

            robot.m_drive.setMovementPowers(lfPower, lrPower, rfPower, rbPower);
        }

        telemetry.addData("loop", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.addData("rawHeading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 180);

        telemetry.update();
    }

    private double getCorrectHeading() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + m_headingOffset;

        while (heading < 0) {
            heading += 2 * Math.PI;
        }

        heading %= (2 * Math.PI);

        return heading;
    }
}