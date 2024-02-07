package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NavXSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;

@TeleOp(name = "IMU Debug", group = "2")
public class IMUDebug extends CommandOpMode {

    NavXSubsystem navXSubsystem;
    ImuSubsystem imuSubsystem;

    TelemetryUpdateSubsystem telemetryUpdateSubsystem;
    @Override
    public void initialize() {
        telemetryUpdateSubsystem = new TelemetryUpdateSubsystem(telemetry);
        navXSubsystem = new NavXSubsystem(hardwareMap,telemetry);
        imuSubsystem = new ImuSubsystem(hardwareMap,telemetry);

        register(navXSubsystem,imuSubsystem,telemetryUpdateSubsystem);
    }
}
