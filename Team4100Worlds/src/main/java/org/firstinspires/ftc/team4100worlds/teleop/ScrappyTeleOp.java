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
    private GamepadEx m_driverOne, m_driverTwo;
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
        imu.resetYaw();
        headingController.setSetPoint(Math.PI);

        schedule(new InstantCommand(() -> {
            robot.m_intake.raise();
            robot.m_intake.back();
            robot.m_intake.stop();
            robot.m_conveyor.stop();
            robot.m_outtake.lower();
            robot.m_outtake.back();
        }));

        // Initialize gamepads
        m_driverOne = new GamepadEx(gamepad1);
        m_driverTwo = new GamepadEx(gamepad2);

        // Initialize slide top position tracker
        m_slideTopPos = robot.m_lift.getPosition() + 600;

        /* Driver One */

        // Drive Method
        m_driverOne.getGamepadButton(GamepadKeys.Button.BACK)
            .whenPressed(new InstantCommand(imu::resetYaw));
        m_driverOne.getGamepadButton(GamepadKeys.Button.START)
            .whenPressed(new InstantCommand(() -> isFieldCentric = !isFieldCentric));

        // Speed
        m_driverOne.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(() -> m_speed = 0.5));
        m_driverOne.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(new InstantCommand(() -> m_speed = 1));

        // Lift
        m_driverOne.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> robot.m_lift.setPosition(0)),
                new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                new InstantCommand(robot.m_outtake::lower)
            ));

        m_driverOne.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> robot.m_lift.setPosition(m_slideTopPos)),
                new InstantCommand(robot.m_outtake::extend)
            ));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(new InstantCommand(() -> {
                if (robot.m_lift.getTargetPosition() <= 250) {
                    robot.m_outtake.lower();
                }

                robot.m_lift.setPositionRel(-160);
            }));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(new InstantCommand(() -> {
                if (robot.m_lift.getTargetPosition() > m_slideTopPos) {
                    m_slideTopPos = robot.m_lift.getTargetPosition();
                }

                if (robot.m_lift.getTargetPosition() >= 250) {
                    robot.m_outtake.extend();

                } else {
                    robot.m_outtake.lower();
                }

                robot.m_lift.setPositionRel(160);
            }));

        // Intake Grabber
        m_driverOne.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        robot.m_intake.lower();
                        robot.m_intake.back();
                    }),
                    new WaitCommand(450),
                    new InstantCommand(() -> robot.m_intake.grab()),
                    new WaitCommand(150),
                    new SequentialCommandGroup(
                        new InstantCommand(() -> robot.m_intake.backOne()),
                        new WaitCommand(150),
                        new InstantCommand(() -> robot.m_intake.backTwo()),
                        new WaitCommand(150),
                        new InstantCommand(() -> robot.m_intake.grab())
                    ),
                    new WaitCommand(150),
                    new SequentialCommandGroup(
                        new InstantCommand(() -> robot.m_intake.backOne()),
                        new WaitCommand(150),
                        new InstantCommand(() -> robot.m_intake.backTwo())
                    )
                )
            );

        m_driverOne.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(new InstantCommand(() -> {
                robot.m_intake.raise();
                robot.m_intake.back();
            }));

        // Dropper
        m_driverOne.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(new InstantCommand(() -> robot.m_outtake.drop()))
            .whenReleased(new InstantCommand(() -> robot.m_outtake.back()));

        // Conveyor + Intake
        new Trigger(() -> m_driverOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
            .whenActive(new ParallelCommandGroup(
                new InstantCommand(robot.m_intake::spit),
                new InstantCommand(robot.m_conveyor::down)
            ))
            .whenInactive(new ConditionalCommand(
                new InstantCommand(),
                new ParallelCommandGroup(
                    new InstantCommand(robot.m_intake::stop),
                    new InstantCommand(robot.m_conveyor::stop)
                ),
                () -> robot.m_intake.isSucking()
            ));
        new Trigger(() -> m_driverOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
            .whenActive(new ParallelCommandGroup(
                new InstantCommand(robot.m_intake::suck),
                new InstantCommand(robot.m_conveyor::up)
            ))
            .whenInactive(new ConditionalCommand(
                new InstantCommand(),
                new ParallelCommandGroup(
                    new InstantCommand(robot.m_intake::stop),
                    new InstantCommand(robot.m_conveyor::stop)
                ),
                () -> robot.m_intake.isSpitting()
            ));

        /* Driver Two */

        // Heading Lock
        m_driverTwo.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(() -> isHeadingLock = !isHeadingLock));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(new InstantCommand(() -> {
                double heading = getCorrectHeading();
                m_headingOffset = 2 * Math.PI - heading;
            }));

        // Safety
        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(new InstantCommand(() -> robot.m_outtake.setRelExtendPos(0.05)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(new InstantCommand(() -> robot.m_outtake.setRelExtendPos(-0.05)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new InstantCommand(() -> robot.m_plane.setRelPos(-0.05)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(new InstantCommand(() -> robot.m_plane.setRelPos(0.05)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.BACK)
            .whenPressed(new InstantCommand(() -> robot.m_lift.reset()));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(new InstantCommand(() -> robot.m_outtake.setRelDropperPos(-0.05)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(new InstantCommand(() -> robot.m_outtake.setRelDropperPos(0.05)));

        // Plane
        m_driverTwo.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(new InstantCommand(robot.m_plane::back));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(new InstantCommand(robot.m_plane::launch));
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

        if (robot.m_lift.getPosition() > m_slideTopPos) {
            m_slideTopPos = robot.m_lift.getPosition();
        }

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
        telemetry.addData("dropper", robot.m_outtake.getDropperPos());
        telemetry.addData("intakeEX", robot.m_intake.getExPos());
        telemetry.addData("outtakeEX", robot.m_outtake.getExPos());
        telemetry.addData("plane", robot.m_plane.getPos());
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