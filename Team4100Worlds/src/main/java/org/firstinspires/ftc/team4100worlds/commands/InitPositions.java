package org.firstinspires.ftc.team4100worlds.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100worlds.subsystem.Intake;
import org.firstinspires.ftc.team4100worlds.subsystem.Lift;
import org.firstinspires.ftc.team4100worlds.subsystem.Outtake;

public class InitPositions extends CommandBase {
    private final Outtake m_outtake;

    public InitPositions(Outtake outtake, Intake intake) {
        m_outtake = outtake;

        addRequirements(outtake, intake);
    }

    @Override
    public void execute() {
        m_outtake.back();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
