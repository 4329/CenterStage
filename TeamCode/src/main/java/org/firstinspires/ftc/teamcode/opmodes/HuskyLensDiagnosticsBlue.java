package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.HuskylensDetectCommand;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "HuskyLens check - BLUE", group = "diagnostics")
public class HuskyLensDiagnosticsBlue extends CommandOpMode {
    private HuskyLensSubsystem huskyLensSubsystem;
    @Override
    public void initialize() {
        huskyLensSubsystem = new HuskyLensSubsystem(hardwareMap, telemetry);

        HuskylensDetectCommand detectCommand = new HuskylensDetectCommand(huskyLensSubsystem, telemetry, Alliance.BLUE);
        huskyLensSubsystem.setDefaultCommand(detectCommand);
        // will this reschedule the default command? - I think so...
    }
}
