package org.firstinspires.ftc.team4100worlds.tests;

import static org.firstinspires.ftc.team4100worlds.ScrappyConstants.DISTANCE_SENSOR_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team4100worlds.subsystem.Sensors;

public class AngleErrorTest extends LinearOpMode {
    private Sensors sensors;

    @Override
    public void runOpMode() {
        sensors = new Sensors(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double dL = sensors.getBackLeftDistance();
            double dR = sensors.getBackRightDistance();

            double factor = 1;

            if (dR > dL) {
                factor = -1;
            }

            double error = factor * Math.atan2(Math.abs(dL - dR), DISTANCE_SENSOR_WIDTH);

            // safety
            if (Math.abs(error) > Math.toRadians(30)) {
                error = 0;
            }

            telemetry.addData("left", dL);
            telemetry.addData("right", dR);
            telemetry.addData("error (deg)", Math.toDegrees(error));
            telemetry.update();
        }
    }
}
