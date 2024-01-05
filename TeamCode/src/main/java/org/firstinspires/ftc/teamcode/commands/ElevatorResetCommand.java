package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ElevatorResetCommand extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;
    private final Telemetry telemetry;
    private boolean downed = false;
    private boolean done = false;


    public ElevatorResetCommand(ElevatorSubsystem elevatorSubsystem, Telemetry telemetry) {

        this.elevatorSubsystem = elevatorSubsystem;
        this.telemetry = telemetry;

    }

    @Override
    public void initialize() {

        Log.i("elevatorReset", "INITIALIZE1234567890");
        elevatorSubsystem.switchPower();
        downed = false;
        done = false;

    }

    @Override
    public void execute() {

        if (!downed) {

            Log.i("elevatorReset", "1NOT downed");
            if (!elevatorSubsystem.isDown()) {
                Log.i("elevatorReset", "2elevatorSubsystem NOT down");

                elevatorSubsystem.goDown();
            } else {
                Log.i("elevatorReset", "3elevatorSubsystem IS down");

                downed = true;
                elevatorSubsystem.stop();

            }
        } else {
            Log.i("elevatorReset", "4IS downed");

            if (elevatorSubsystem.isDown()) {
                Log.i("elevatorReset", "5elevatorSubsystem IS downDowned");

                elevatorSubsystem.goUp();
            } else {

                Log.i("elevatorReset", "6elevatorSubsystem IS !DownedbutUP");

                elevatorSubsystem.stop();
                done = true;

            }
        }


    }

    @Override
    public void end(boolean interrupted) {
        Log.i("elevatorReset", "ENDENDEND");
        elevatorSubsystem.switchPosition();

    }

    @Override
    public boolean isFinished() {

        return done;
    }
}