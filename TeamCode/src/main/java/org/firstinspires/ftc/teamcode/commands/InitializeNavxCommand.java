package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;

public class InitializeNavxCommand extends CommandBase {

    private ImuSubsystem imuSubsystem;
    private Telemetry telemetry;

    public InitializeNavxCommand(ImuSubsystem imuSubsystem, Telemetry telemetry) {
        this.imuSubsystem = imuSubsystem;
        this.telemetry = telemetry;
        addRequirements(imuSubsystem);
    }

    @Override
    public void initialize() {
        telemetry.log().add("Navx Gyro calibrating.");
    }

    @Override
    public void execute() {
         telemetry.addData("calibrating", "the navx");
    }

    @Override
    public void end(boolean interrupted) {
        telemetry.log().clear();
        if (interrupted) {
            telemetry.log().add("Gyro timed out while trying to calibrate");
        }
        else {
            telemetry.log().add("Gyro calibrated");
        }
    }

    @Override
    public boolean isFinished() {
        return !imuSubsystem.isCalibrating();
    }
}
