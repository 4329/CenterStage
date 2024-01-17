package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

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
        Log.i(this.getName(), "initialize: Moving to position "+armPosition);
    }


    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        Log.i(this.getName(), "isFinished: Arm Position "+ armSubsystem.getPosition());
        Log.i(this.getName(), "isFinished: armAtPosition "+armSubsystem.armAtPosition());
        return armSubsystem.armAtPosition();
    }
}