package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

public class UnInstantCommand extends CommandBase {
    private final Runnable m_toRun;

    public UnInstantCommand(Runnable toRun, Subsystem... requirements) {
        m_toRun = toRun;

        addRequirements(requirements);
    }

    @Override
    public void execute() {
        m_toRun.run();
    }

    @Override
    public final boolean isFinished() {
        return true;
    }

}
