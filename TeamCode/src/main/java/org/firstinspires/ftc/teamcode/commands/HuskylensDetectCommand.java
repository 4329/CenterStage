package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;

public class HuskylensDetectCommand extends CommandBase {

    private HuskyLensSubsystem huskyLensSubsystem;
    private HuskyLens.Block lastBlock;
    private Telemetry telemetry;

    public HuskylensDetectCommand(HuskyLensSubsystem huskyLensSubsystem, Telemetry telemetry) {
        this.huskyLensSubsystem = huskyLensSubsystem;
        this.telemetry = telemetry;

    }

    @Override
    public void execute() {
        lastBlock = huskyLensSubsystem.detectBlocks();
        telemetry.addLine("pixelPosition is" + huskyLensSubsystem.getPixelPosition());

    }

    @Override
    public boolean isFinished() {
        return lastBlock != null;
    }


}
