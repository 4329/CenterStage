package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.HuskylensDetectCommand;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;

@TeleOp(name = "HuskyTest", group = "1")
public class HuskyTest extends CommandOpMode {

    private HuskyLensSubsystem huskyLensSubsystem;
    private TelemetryUpdateSubsystem telemetryUpdateSubsystem;

    @Override
    public void initialize() {
        telemetryUpdateSubsystem = new TelemetryUpdateSubsystem(telemetry);
        huskyLensSubsystem = new HuskyLensSubsystem(hardwareMap, telemetry);
        Command see = new HuskylensDetectCommand(huskyLensSubsystem, telemetry);
        schedule(see);
        register(telemetryUpdateSubsystem);
    }

}
