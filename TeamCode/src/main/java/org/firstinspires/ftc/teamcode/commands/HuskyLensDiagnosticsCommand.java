package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class HuskyLensDiagnosticsCommand extends HuskylensDetectCommand {

    private final HuskyLensSubsystem huskyLensSubsystem;
    private final Telemetry telemetry;
    private final Alliance alliance;

    public HuskyLensDiagnosticsCommand(HuskyLensSubsystem huskyLensSubsystem, Telemetry telemetry, Alliance alliance) {
        super(huskyLensSubsystem, telemetry, alliance);
        this.huskyLensSubsystem = huskyLensSubsystem;
        this.telemetry = telemetry;
        this.alliance = alliance;
    }

    @Override
    public boolean isFinished() {
        if (count % 15 == 0) {
            huskyLensSubsystem.clearMessages();
            summary();
            count = 0;
            lastBlocks.clear();
        }
        return false;
    }
}
