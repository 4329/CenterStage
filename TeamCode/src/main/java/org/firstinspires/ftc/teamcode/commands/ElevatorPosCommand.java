package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;

import java.util.function.DoubleSupplier;

public class ElevatorPosCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private ElevatorSubsystem es;
    private  Telemetry telemetry;
    private ElevatorPosition elevatorPosition;
    public ElevatorPosCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPosition elevatorPosition, Telemetry telemetry) {

        this.elevatorPosition = elevatorPosition;
        this.elevatorSubsystem = elevatorSubsystem;
        this.telemetry = telemetry;
        addRequirements(elevatorSubsystem);
    }


    @Override
    public void initialize() {
        elevatorSubsystem.goToPosition(elevatorPosition);
    }
    @Override
    public void end(boolean interrupted) {

        elevatorSubsystem.stop();

    }

    @Override
    public boolean isFinished() {

        return elevatorSubsystem.uThereYet();

    }
}