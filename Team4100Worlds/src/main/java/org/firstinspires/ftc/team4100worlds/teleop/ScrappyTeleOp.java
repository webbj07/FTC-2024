package org.firstinspires.ftc.team4100worlds.teleop;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team4100worlds.commands.ProfiledLiftCommand;

@TeleOp
public class ScrappyTeleOp extends ScrappyTeleOpBase {
    private GamepadEx m_driverOne, m_driverTwo;
    private int m_slideTopPos = 0;
    private double m_speed = 1;
    private boolean isFieldCentric = false;
    private ElapsedTime elapsedTime = new ElapsedTime();

    public ScrappyTeleOp() {
        super(null, null);
    }

    @Override
    public void initTeleOp() {
        imu.resetYaw();
        schedule(new InstantCommand(() -> {
            robot.m_outtake.back();
            robot.m_outtake.lower();
            robot.m_intake.raise();
            robot.m_intake.back();
        }));

        // Initialize gamepads
        m_driverOne = new GamepadEx(gamepad1);
        m_driverTwo = new GamepadEx(gamepad2);

        // Initialize slide top position tracker
        m_slideTopPos = robot.m_lift.getPosition() + 600;

        /* Driver One */
        // Speed
        m_driverOne.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> m_speed = 0.35));
        m_driverOne.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> m_speed = 1));
        m_driverOne.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> imu.resetYaw()));
        m_driverOne.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> isFieldCentric = !isFieldCentric));

        // Lift
        m_driverOne.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SequentialCommandGroup(
                    new InstantCommand(() -> robot.m_lift.gotoPos(0)),
                    new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(0)),
                    new InstantCommand(robot.m_outtake::lower)
                ));

        m_driverOne.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ParallelCommandGroup(
                        new InstantCommand(() -> robot.m_lift.gotoPos(m_slideTopPos)),
                        new InstantCommand(robot.m_outtake::extend)
                ));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> robot.m_lift.gotoPosRel(-160)));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> robot.m_lift.gotoPosRel(160)));

        // Intake Grabber
        m_driverOne.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(robot.m_intake::lower));

        m_driverOne.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(robot.m_intake::raise));

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
        // Plane
//        m_driverTwo.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(new InstantCommand(() -> robot.m_outtake.setRelExtendPos(0.05)));
//        m_driverTwo.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(new InstantCommand(() -> robot.m_outtake.setRelExtendPos(-0.05)));
//        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new InstantCommand(() -> robot.m_plane.setRelPos(-0.05)));
//        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new InstantCommand(() -> robot.m_plane.setRelPos(0.05)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(robot.m_plane::back));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(robot.m_plane::launch));
//        m_driverTwo.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new InstantCommand(() -> robot.m_lift.reset()));
//        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(() -> robot.m_outtake.setRelDropperPos(-0.05)));
//        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> robot.m_outtake.setRelDropperPos(0.05)));
    }
    @Override
    public void run() {
        super.run();

        if (robot.m_lift.getPosition() > m_slideTopPos) {
            m_slideTopPos = robot.m_lift.getPosition();
        }

        if (isFieldCentric) {
            this.driveFieldCentric(gamepad1, m_speed);
        } else {
            this.drive(gamepad1, m_speed);
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
}