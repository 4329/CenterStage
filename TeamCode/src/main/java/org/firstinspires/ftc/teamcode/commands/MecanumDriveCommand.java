package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

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
    private BooleanSupplier slowBoost;
    private Telemetry telemetry;
    private static double REGULAR_MOTION_DIVISOR = 3.0;
    private static double SLOW_MOTION_DIVISOR = 6.0;

    public MecanumDriveCommand(MecanumDriveSubsystem mecanumDriveSubsystem,
                               DoubleSupplier forwardDrive,
                               DoubleSupplier rotateDrive,
                               DoubleSupplier strafeDrive,
                               BooleanSupplier speeedBooost,
                               BooleanSupplier slowBoost,
                               Telemetry telemetry) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.forwardDrive = forwardDrive;
        this.rotateDrive = rotateDrive;
        this.strafeDrive = strafeDrive;
        this.speedBooost = speeedBooost;
        this.slowBoost = slowBoost;
        this.telemetry = telemetry;
        addRequirements(mecanumDriveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        telemetry.addData("speedBooost", speedBooost.getAsBoolean());
        if (speedBooost.getAsBoolean()) {
            mecanumDriveSubsystem.drive(
                    forwardDrive.getAsDouble(),
                    rotateDrive.getAsDouble(),
                    strafeDrive.getAsDouble());
        } else if (slowBoost.getAsBoolean()) {
            mecanumDriveSubsystem.drive(
                    forwardDrive.getAsDouble() / SLOW_MOTION_DIVISOR,
                    rotateDrive.getAsDouble() / SLOW_MOTION_DIVISOR,
                    strafeDrive.getAsDouble() / SLOW_MOTION_DIVISOR);

        } else {

            mecanumDriveSubsystem.drive(
                    forwardDrive.getAsDouble() / REGULAR_MOTION_DIVISOR,
                    rotateDrive.getAsDouble() / REGULAR_MOTION_DIVISOR,
                    strafeDrive.getAsDouble() / REGULAR_MOTION_DIVISOR);
        }
    }
}