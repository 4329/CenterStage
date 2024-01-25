package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.HuskyLensDiagnosticsCommand;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "HuskyLens check - RED", group = "diagnostics")
public class HuskyLensDiagnosticsRed extends CommandOpMode {
    private HuskyLensSubsystem huskyLensSubsystem;
    private TelemetryUpdateSubsystem telemetryUpdateSubsystem;
    @Override
    public void initialize() {
        huskyLensSubsystem = new HuskyLensSubsystem(hardwareMap, telemetry);
        telemetryUpdateSubsystem = new TelemetryUpdateSubsystem(telemetry);

        Command detectCommand = new HuskyLensDiagnosticsCommand(huskyLensSubsystem, telemetry, Alliance.RED);
        huskyLensSubsystem.setDefaultCommand(detectCommand);

        register(telemetryUpdateSubsystem);
    }
}
