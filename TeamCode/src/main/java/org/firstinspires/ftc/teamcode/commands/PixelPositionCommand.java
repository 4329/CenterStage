package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;
import org.firstinspires.ftc.teamcode.util.PixelPosition;

public class PixelPositionCommand extends CommandBase {


    private final HuskyLensSubsystem huskyLensSubsystem;
    private final Telemetry telemetry;

    public PixelPositionCommand(HuskyLensSubsystem huskyLensSubsystem, Telemetry telemetry) {


        this.telemetry = telemetry;
        this.huskyLensSubsystem = huskyLensSubsystem;
        addRequirements(huskyLensSubsystem);

    }

    @Override
    public void execute() {
        huskyLensSubsystem.getPixelPosition();
        telemetry.addLine("Position is " + huskyLensSubsystem.getPixelPosition());

    }
}
