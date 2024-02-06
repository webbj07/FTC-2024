package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.RunAction;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalization;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class ScrappyTeleOp extends ScrappyTeleOpBase {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private GamepadEx m_driverOne, m_driverTwo;
    private int m_slideTopPos = 0;
    private double m_speed = 1;
    private boolean isFieldCentric = true;

    public ScrappyTeleOp() {
        super(null, null);
    }

    @Override
    public void initTeleOp() {
        initAprilTagDetection();

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
        m_driverOne.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    visionPortal.setProcessorEnabled(aprilTagProcessor, true);

                    int att = 0;

                    Vector2d aprilTagPose = getAprilTagDetection();
                    while (aprilTagPose == null && att++ < 5) {
                        sleep(20);
                        aprilTagPose = getAprilTagDetection();
                    }

                    visionPortal.setProcessorEnabled(aprilTagProcessor, false);

                    if (aprilTagPose != null) {
                        Action gotoTag = robot.m_drive.actionBuilder(robot.m_drive.pose)
                                .strafeToLinearHeading(new Vector2d(aprilTagPose.x, aprilTagPose.y), Math.PI)
                                .build();

                        schedule(new RunAction(gotoTag));
                    }
                }));

        // Speed
        m_driverOne.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> m_speed = 0.16));
        m_driverOne.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> m_speed = 1));

        // Lift
        m_driverOne.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SequentialCommandGroup(
                    new InstantCommand(robot.m_lift::toInitial),
                    new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(robot.m_lift.getTargetPosition())),
                    new InstantCommand(() -> robot.m_outtake.setExtPos(0.18))
                ));

        m_driverOne.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    robot.m_lift.setPosition(m_slideTopPos);
                    robot.m_outtake.setExtPos(0.53);
                } ));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> robot.m_lift.setRelativePosition(-160)));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (robot.m_lift.getTargetPosition() > m_slideTopPos) {
                        m_slideTopPos = robot.m_lift.getTargetPosition();
                    }

                    if (robot.m_lift.getTargetPosition() >= 250) {
                        robot.m_outtake.setExtPos(0.53);
                    } else {
                        robot.m_outtake.setExtPos(0.18);
                    }

                    robot.m_lift.setRelativePosition(160);
                }));

        // Intake Grabber
        m_driverOne.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(robot.m_intake::exDown));

        m_driverOne.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(robot.m_intake::exUp));

        // Dropper
        m_driverOne.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    robot.m_outtake.drop();
                }))
                .whenReleased(new InstantCommand(() -> {
                    robot.m_outtake.back();
                }));

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
        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> robot.m_intake.setGrabOnePos(-0.05)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> robot.m_intake.setGrabOnePos(0.05)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> robot.m_intake.setGrabTwoPos(-0.05)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> robot.m_intake.setGrabTwoPos(0.05)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(robot.m_plane::back));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(robot.m_plane::launch));


    }
    @Override
    public void run() {
        super.run();

        if (isFieldCentric) {
            Rotation2d gyroAngle = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            this.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gyroAngle.toDouble());
        } else {
            this.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        telemetry.addData("one", robot.m_intake.m_grabberOne.getPosition());
        telemetry.addData("two", robot.m_intake.m_grabberTwo.getPosition());
        telemetry.update();
    }

    private void initAprilTagDetection() {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
    }

    private Vector2d getAprilTagDetection() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        if (currentDetections.size() > 0) {
            AprilTagDetection detection = currentDetections.get(0);

            double heading = robot.m_drive.pose.heading.toDouble();
            Pose2d estimatedPos = AprilTagLocalization.getRobotPositionFromTag(detection, heading, true);
            robot.m_drive.pose = estimatedPos;

            return AprilTagLocalization.getTagPosition(detection);
        }

        return null;
    }
}