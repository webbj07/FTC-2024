package org.firstinspires.ftc.team4100worlds.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team4100worlds.ScrappyCore;
import org.firstinspires.ftc.team4100worlds.ScrappyConstants;

public abstract class ScrappyTeleOpBase extends CommandOpMode {
    private final ScrappyConstants.AllianceType m_allianceType;
    private final ScrappyConstants.AllianceSide m_allianceSide;

    protected ScrappyCore robot;
    protected IMU imu;

    public ScrappyTeleOpBase(ScrappyConstants.AllianceType allianceType, ScrappyConstants.AllianceSide allianceSide) {
        m_allianceType = allianceType;
        m_allianceSide = allianceSide;
    }

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new ScrappyCore(hardwareMap, m_allianceType, m_allianceSide);
        robot.m_drive.setAuto(false);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(ScrappyConstants.CONTROL_HUB_ORIENTATION);
        imu.initialize(parameters);

        initTeleOp();
    }

    protected void drive(Gamepad gamepad1, double speed) {
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double leftFrontPower = Range.clip((y + x + rx), -1, 1) * speed;
        double leftRearPower = Range.clip((y - x + rx), -1, 1) * speed;
        double rightFrontPower = Range.clip((y - x - rx), -1, 1) * speed;
        double rightRearPower = Range.clip((y + x - rx), -1, 1) * speed;

        robot.m_drive.setMovementPowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
    }

    protected void driveFieldCentric(Gamepad gamepad1, double speed) {
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftRearPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightRearPower = (rotY + rotX - rx) / denominator;

        robot.m_drive.setMovementPowers(leftFrontPower * speed, leftRearPower * speed, rightFrontPower * speed, rightRearPower * speed);
    }

    public abstract void initTeleOp();
}
