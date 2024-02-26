package org.firstinspires.ftc.team4100worlds.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100worlds.subsystem.Intake;
import org.firstinspires.ftc.team4100worlds.subsystem.Lift;
import org.firstinspires.ftc.team4100worlds.subsystem.Outtake;

public class InitPositions extends CommandBase {
    private final Lift m_lift;
    private final Outtake m_outtake;

    public InitPositions(Lift lift, Outtake outtake, Intake intake) {
        m_lift = lift;
        m_outtake = outtake;

        addRequirements(lift, outtake, intake);
    }

    @Override
    public void execute() {
        m_lift.toInitial();
        m_outtake.back();
//        m_intake.back();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
