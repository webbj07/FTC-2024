package org.firstinspires.ftc.team4100worlds.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.team4100worlds.subsystem.Lift;
import org.firstinspires.ftc.teamcode.util.NanoClock;

@Config
public class ProfiledLiftCommand extends CommandBase {
    private final Lift lift;
    private PIDController liftController;
    private MotionProfile profile;
    private int targetPosition;
    private int currentPosition;
    private boolean isRelative = false;
    private final NanoClock clock = NanoClock.system();
    private double profileStart;

    public ProfiledLiftCommand(Lift lift, int targetPosition) {
        this.lift = lift;
        this.targetPosition = targetPosition;

//        liftController = new PIDController(Lift.PID.kP, Lift.PID.kI, Lift.PID.kD);
//        addRequirements(lift);
    }

    public ProfiledLiftCommand(Lift lift, int targetPosition, boolean isRelative) {
        this.lift = lift;
        this.targetPosition = targetPosition;
        this.isRelative = isRelative;

//        liftController = new PIDController(Lift.PID.kP, Lift.PID.kI, Lift.PID.kD);
//        addRequirements(lift);
    }

    @Override
    public void initialize() {
        targetPosition = isRelative ? targetPosition + lift.getPosition() : targetPosition;
//        profile = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(lift.getPosition(), lift.getVelocity()),
//                new MotionState(targetPosition, 0),
//                Lift.MAX_VEL,
//                Lift.MAX_ACCEL,
//                0
//        );
//
//        liftController.reset();
//        profileStart = clock.seconds();
        lift.gotoPos(targetPosition);
    }

    @Override
    public void execute() {
//        double profileTime = clock.seconds() - profileStart;
//        currentPosition = lift.getPosition();
//
//        MotionState targetMotionState = profile.get(profileTime);
//        double feedback = liftController.calculate(currentPosition, (int) targetMotionState.getX());
//        double power = feedback + Lift.kG;
//
//        lift.setPower(power);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(targetPosition - lift.getPosition()) <= Lift.LIFT_TOLERANCE) {
//            lift.setPower(0);
            return true;
        }
        return false;
    }
}