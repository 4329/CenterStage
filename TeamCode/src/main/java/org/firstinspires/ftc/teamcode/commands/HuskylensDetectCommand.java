package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class HuskylensDetectCommand extends CommandBase {

    private HuskyLensSubsystem huskyLensSubsystem;
    private HuskyLens.Block lastBlock;
    private Telemetry telemetry;
    private Alliance alliance;

    public HuskylensDetectCommand(HuskyLensSubsystem huskyLensSubsystem, Telemetry telemetry, Alliance alliance) {
        this.huskyLensSubsystem = huskyLensSubsystem;
        this.telemetry = telemetry;
        this.alliance = alliance;

    }

    @Override
    public void execute() {
        lastBlock = huskyLensSubsystem.detectBlocks(alliance);
        telemetry.addLine("pixelPosition is" + huskyLensSubsystem.getPixelPosition());

    }

    @Override
    public boolean isFinished() {
        return lastBlock != null;
    }


}
