package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;

public class HuskylensDetectCommand extends CommandBase {

    private HuskyLensSubsystem huskyLensSubsystem;
    private HuskyLens.Block lastBlock;

    public HuskylensDetectCommand(HuskyLensSubsystem huskyLensSubsystem) {
        this.huskyLensSubsystem = huskyLensSubsystem;

    }

    @Override
    public void execute() {
        lastBlock = huskyLensSubsystem.detectBlocks();
    }

    @Override
    public boolean isFinished() {
        return lastBlock != null;
    }


}
