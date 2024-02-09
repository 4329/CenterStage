package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

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
        this.frcPid = new FrcPidController(0.035, .00, 0.00125); //kd was 0.000075
        addRequirements(mecanumDriveSubsystem);
    }

    @Override
    public void initialize() {
        frcPid.setSetpoint(DesiredAngle);
        frcPid.setTolerance(0.2);
        frcPid.enableContinuousInput(-180, 180);
        Log.i("turnCommand", "turnstarting");

    }

    @Override
    public void execute() {

        telemetry.addLine("is going");
        double heading = imu.getHeading();
        double output = frcPid.calculate(heading);
        output = Range.clip(output,-.5,.5);
        drive.drive(0, -output, 0);
        Log.i("turnCommand", "desired angle, heading, output " + "(" + DesiredAngle + ", " + heading + ", " + output + ")");
    }

    @Override
    public void end(boolean interrupted) {

        drive.stop();
        Log.i("turnCommand", "TURNFINISHED");


    }

    @Override
    public boolean isFinished() {

        return frcPid.atSetpoint();
    }
}
