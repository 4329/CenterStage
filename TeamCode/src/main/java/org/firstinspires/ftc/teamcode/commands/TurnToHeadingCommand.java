package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class TurnToHeadingCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final ImuSubsystem imu;
    private final Telemetry telemetry;
    private final double DesiredAngle;

    public TurnToHeadingCommand(MecanumDriveSubsystem mecanumDriveSubsystem, ImuSubsystem imuSubsystem, Telemetry telemetry, double DesiredAngle) {

        this.drive = mecanumDriveSubsystem;
        this.imu = imuSubsystem;
        this.telemetry = telemetry;
        this.DesiredAngle = DesiredAngle;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}
