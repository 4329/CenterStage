package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.HuskylensDetectCommand;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "HuskyLens check - RED", group = "diagnostics")
public class HuskyLensDiagnosticsRed extends CommandOpMode {
    private HuskyLensSubsystem huskyLensSubsystem;
    @Override
    public void initialize() {
        huskyLensSubsystem = new HuskyLensSubsystem(hardwareMap, telemetry);

        HuskylensDetectCommand detectCommand = new HuskylensDetectCommand(huskyLensSubsystem, telemetry, Alliance.RED);
        huskyLensSubsystem.setDefaultCommand(detectCommand);
        // will this reschedule the default command? - I think so...
    }
}
