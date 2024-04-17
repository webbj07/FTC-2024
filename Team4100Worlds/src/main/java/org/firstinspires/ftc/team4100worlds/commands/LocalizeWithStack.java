package org.firstinspires.ftc.team4100worlds.commands;

import static org.firstinspires.ftc.team4100worlds.ScrappyConstants.FRONT_DISTANCE_SENSOR_POSITION;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100worlds.autonomous.ScrappyAutoBase;

import java.util.ArrayList;

public class LocalizeWithStack extends CommandBase {
    public static double DRIVE_ERROR_THRESHOLD = 2;
    public static double STRAFE_ERROR_THRESHOLD = 2;
    public static int STRAFE_ERROR_ATTEMPTS = 10;
    public static int STRAFE_ERROR_ATTEMPTS_MIN = 10;
    private final ScrappyAutoBase m_base;
    private boolean m_finished = false;

    public LocalizeWithStack(ScrappyAutoBase base) {
        m_base = base;
    }

    @Override
    public void initialize() {
        ArrayList<Double> errorList = new ArrayList<>();

        int attempts = 0;

        while (errorList.size() < STRAFE_ERROR_ATTEMPTS_MIN || attempts++ < STRAFE_ERROR_ATTEMPTS) {
            Double error = m_base.stackProcessor.getStrafeError();
            if (error != null) {
                errorList.add(error);
            }
        }

        if (attempts >= STRAFE_ERROR_ATTEMPTS) {
            m_finished = true;
            return;
        }

        double strafeError = 0;

        for (Double err : errorList) {
            strafeError += err;
        }

        strafeError /= errorList.size();

        if (strafeError >= STRAFE_ERROR_THRESHOLD) {
            strafeError = 0;
        }

        double driveError = (-71 + m_base.robot.m_sensors.getFrontDistanceAsync() + FRONT_DISTANCE_SENSOR_POSITION.getY()) - m_base.robot.m_drive.getPose().getX();

        if (driveError >= DRIVE_ERROR_THRESHOLD) {
            driveError = 0;
        }

        m_base.robot.m_drive.poseUpdater.setXOffset(driveError);
        m_base.robot.m_drive.poseUpdater.setYOffset(strafeError);

        m_finished = true;
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
