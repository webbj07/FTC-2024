package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Dropper;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class InitPositions extends CommandBase {
    private final Lift m_lift;
    private final Dropper m_dropper;
    private final Intake m_intake;

    public InitPositions(Lift lift, Dropper dropper, Intake intake) {
        m_lift = lift;
        m_dropper = dropper;
        m_intake = intake;

        addRequirements(lift, dropper, intake);
    }

    @Override
    public void execute() {
        m_lift.toInitial();
        m_dropper.back();
        m_intake.back();
    }

    @Override
    public boolean isFinished() {
        return m_lift.isWithinTolerance(m_lift.getTargetPosition());
    }
}
