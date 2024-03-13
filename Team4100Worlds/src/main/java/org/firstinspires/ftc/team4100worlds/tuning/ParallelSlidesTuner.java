package org.firstinspires.ftc.team4100worlds.tuning;

import static org.firstinspires.ftc.team4100worlds.subsystem.Lift.MAX_ACCEL;
import static org.firstinspires.ftc.team4100worlds.subsystem.Lift.MAX_VEL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.util.NanoClock;
import org.firstinspires.ftc.teamcode.util.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.util.profiling.MotionProfileGenerator;
import org.firstinspires.ftc.teamcode.util.profiling.MotionState;

@Config
@TeleOp
public final class ParallelSlidesTuner extends LinearOpMode {
    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }
    public static PIDCoefficients PID = new PIDCoefficients(0, 0, 0);
    public static double kV = 0.000495, kA = 0.0001, kG = 0;
    public static int targetPos = 1700;
    private DcMotorEx leftSlide, rightSlide;
    private PIDController controller;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "RSlide");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Mode mode = Mode.TUNING_MODE;

        double lastKp = PID.p;
        double lastKi = PID.i;
        double lastKd = PID.d;
        controller = new PIDController(lastKp, lastKi, lastKd);

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();

        while (!isStopRequested()) {

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                    }

                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    int currentPos = (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
                    int error = targetPos - currentPos;

                    MotionState targetMotionState = activeProfile.get(profileTime);
                    double feedback = controller.calculate(currentPos, (int) targetMotionState.getX());
                    double feedforward = (kV * targetMotionState.getV()) + (kA * targetMotionState.getA());
                    double power = feedback + feedforward + kG;

                    leftSlide.setPower(power);
                    rightSlide.setPower(power);

                    telemetry.addData("currentPos", currentPos);
                    telemetry.addData("targetPos", targetPos);
                    telemetry.addData("error", error);
                    break;
                case DRIVER_MODE:
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(true);
                        profileStart = clock.seconds();
                    }

                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    break;
            }
            if (lastKp != PID.p || lastKi != PID.i || lastKd != PID.d) {
                lastKp = PID.p;
                lastKi = PID.i;
                lastKd = PID.d;
                controller.setPID(lastKp, lastKi, lastKd);
            }

            telemetry.update();
        }
    }

    private static MotionProfile generateProfile(boolean movingUp) {
        MotionState start = new MotionState(movingUp ? 0 : targetPos, 0, 0, 0);
        MotionState goal = new MotionState(movingUp ? targetPos : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL, 0);
    }
}