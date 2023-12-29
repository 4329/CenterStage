package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ElevatorVerticalCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private ElevatorSubsystem es;
    private  Telemetry telemetry;
    private DoubleSupplier elevatorPower;
    private BooleanSupplier reset;
    public ElevatorVerticalCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier elevatorPower,
                                   BooleanSupplier reset,
                                   Telemetry telemetry) {

        this.elevatorPower = elevatorPower;
        this.elevatorSubsystem = elevatorSubsystem;
        this.telemetry = telemetry;
        this.reset = reset;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        telemetry.addLine("Elevator hath runneth!");
        elevatorSubsystem.move(elevatorPower.getAsDouble(), reset.getAsBoolean());
    }




    @Override
    public void end(boolean interrupted) {

    elevatorSubsystem.stop();

    }
}