package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class InitPositions extends CommandBase {
    private final Lift m_lift;
    private final Outtake m_outtake;
    private final Intake m_intake;

    public InitPositions(Lift lift, Outtake outtake, Intake intake) {
        m_lift = lift;
        m_outtake = outtake;
        m_intake = intake;

        addRequirements(lift, outtake, intake);
    }

    @Override
    public void execute() {
        m_lift.toInitial();
        m_outtake.back();
        m_intake.back();
    }

    @Override
    public boolean isFinished() {
        return m_lift.isWithinTolerance(m_lift.getTargetPosition());
    }
}
