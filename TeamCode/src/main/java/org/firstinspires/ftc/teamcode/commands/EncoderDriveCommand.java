package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class EncoderDriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final double forward;
    private final double turn;
    private final double strafe;
    private final double inches;
    public static final double TICKS_PER_REV = 28;
    public static final double MAX_RPM = 6000;
    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = .0529100529; // output (wheel) speed / input (motor) speed
    private Motor.Encoder encoder;
    private int startingTicks;

    public EncoderDriveCommand(MecanumDriveSubsystem mecanumDriveSubsystem, double forward, double turn, double strafe, double inches) {
        this.drive = mecanumDriveSubsystem;
        this.forward = forward;
        this.turn = turn;
        this.strafe = strafe;
        this.inches = inches;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        encoder = drive.getSingleEncoder();
        startingTicks = encoder.getPosition();
    }

    @Override
    public void execute() {
        drive.drive(forward, turn, strafe);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        int larry = Math.abs(encoder.getPosition() - startingTicks);
        double remainingInches = encoderTicksToInches(larry);
        if(remainingInches >= inches){
            return true;
        }
        else{
            return false;
        }
    }
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
