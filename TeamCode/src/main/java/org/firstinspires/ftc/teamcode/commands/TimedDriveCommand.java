package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.concurrent.TimeUnit;

public class TimedDriveCommand extends CommandBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final double strafe;
    private final double turn;
    private final double forward;
    private Timing.Timer timer;
    private final long milliseconds;

    public TimedDriveCommand(MecanumDriveSubsystem mecanumDriveSubsystem, double forward, double turn, double strafe, long milliseconds) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.forward = forward;
        this.turn = turn;
        this.strafe = strafe;
        this.milliseconds = milliseconds;


    }

    @Override
    public void initialize() {
        Log.i("Roboteers4329", "intitianlize happened");
        this.timer = new Timing.Timer(milliseconds, TimeUnit.MILLISECONDS);
        timer.start();
    }

    @Override
    public void execute() {
        Log.i("Roboteers4329", "execute called");
        mecanumDriveSubsystem.drive(forward, turn, strafe);
    }

    @Override
    public void end(boolean interrupted) {
        Log.i("Roboteers4329", "end called");
        mecanumDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        Log.i("Roboteers4329", "is finished called, " + timer.done());
        return timer.done();
    }
}
