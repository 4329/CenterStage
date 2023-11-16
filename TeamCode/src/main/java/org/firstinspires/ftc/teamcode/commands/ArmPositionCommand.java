package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.util.ArmPosition;

public class ArmPositionCommand extends CommandBase {


    private final ArmPosition armPosition;
    private final ArmSubsystem armSubsystem;

    public ArmPositionCommand(ArmSubsystem armSubsystem, ArmPosition armPosition) {


    this.armPosition = armPosition;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    }


    @Override
    public void initialize() {
        armSubsystem.goToPosition(armPosition);
    }


    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.armAtPosition();
    }
}