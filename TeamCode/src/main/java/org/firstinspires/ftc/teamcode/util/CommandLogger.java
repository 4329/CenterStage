package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;

public class CommandLogger {
    public CommandLogger() {
        logCommander();
    }

    private void logCommander() {
        CommandScheduler cmds = CommandScheduler.getInstance();
        cmds.onCommandInitialize((cmd) -> Log.i("command", cmd.getName() + " was initialized"));
        cmds.onCommandFinish((cmd) -> Log.i("command", cmd.getName() + " was finished"));
        cmds.onCommandInterrupt((cmd) -> Log.i("command", cmd.getName() + " was INTERRuPTED"));
    }
}