package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorVerticalCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private ElevatorSubsystem es;
    private  Telemetry telemetry;
    private DoubleSupplier elevatorPower;
    public ElevatorVerticalCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier elevatorPower, Telemetry telemetry) {

        this.elevatorPower = elevatorPower;
        this.elevatorSubsystem = elevatorSubsystem;
        this.telemetry = telemetry;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.move(elevatorPower.getAsDouble());
    }




    @Override
    public void end(boolean interrupted) {

    elevatorSubsystem.stop();

    }
}