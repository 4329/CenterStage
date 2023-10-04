package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private DoubleSupplier forwardDrive;
    private DoubleSupplier rotateDrive;
    private DoubleSupplier strafeDrive;
    private BooleanSupplier speedBooost;
    private Telemetry telemetry;
    private final static double SLOW_MOTION_DIVISOR=3.0;

    public MecanumDriveCommand(MecanumDriveSubsystem mecanumDriveSubsystem,
                               DoubleSupplier forwardDrive,
                               DoubleSupplier rotateDrive,
                               DoubleSupplier strafeDrive,
                               BooleanSupplier speedBooost,
                               Telemetry telemetry) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.forwardDrive = forwardDrive;
        this.rotateDrive = rotateDrive;
        this.strafeDrive = strafeDrive;
        this.speedBooost = speedBooost;
        this.telemetry = telemetry;
        addRequirements(mecanumDriveSubsystem);
    }

    @Override
    public void initialize() {
        telemetry.speak("speedBooost");
    }

    @Override
    public void execute() {
        telemetry.addData("speedBooost",speedBooost.getAsBoolean());
        if (speedBooost.getAsBoolean()) {
            mecanumDriveSubsystem.drive(
                    forwardDrive.getAsDouble(),
                    rotateDrive.getAsDouble(),
                    strafeDrive.getAsDouble());
        }
        else {

            mecanumDriveSubsystem.drive(
                    forwardDrive.getAsDouble()/SLOW_MOTION_DIVISOR,
                    rotateDrive.getAsDouble()/SLOW_MOTION_DIVISOR,
                    strafeDrive.getAsDouble()/SLOW_MOTION_DIVISOR);
        }
    }
}