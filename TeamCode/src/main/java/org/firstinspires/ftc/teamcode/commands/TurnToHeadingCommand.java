package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.FrcPidController;

public class TurnToHeadingCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final ImuSubsystem imu;
    private final Telemetry telemetry;
    private final double DesiredAngle;
    private final FrcPidController frcPid;

    public TurnToHeadingCommand(MecanumDriveSubsystem mecanumDriveSubsystem, ImuSubsystem imuSubsystem, Telemetry telemetry, double DesiredAngle) {

        this.drive = mecanumDriveSubsystem;
        this.imu = imuSubsystem;
        this.telemetry = telemetry;
        this.DesiredAngle = DesiredAngle;
        this.frcPid = new FrcPidController(0.037, 0, 0.000075);
    }

    @Override
    public void initialize() {
        frcPid.setSetpoint(DesiredAngle);
        frcPid.setTolerance(0.1);
        frcPid.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        double output = frcPid.calculate(imu.getHeading());

        drive.drive(0, -output, 0);
    }

    @Override
    public void end(boolean interrupted) {

        drive.stop();
    }

    @Override
    public boolean isFinished() {

        return frcPid.atSetpoint();
    }
}
