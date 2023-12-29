package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;

import java.util.function.BooleanSupplier;

public class FireZeMisslizCommand extends CommandBase {
    private final DroneSubsystem droneSubsystem;
    private final BooleanSupplier driver;
    private final BooleanSupplier opratr;

    public FireZeMisslizCommand(DroneSubsystem droneSubsystem, BooleanSupplier driver, BooleanSupplier opratr) {
        this.droneSubsystem = droneSubsystem;
        this.driver = driver;
        this.opratr = opratr;
        addRequirements(droneSubsystem);
    }

    @Override
    public void execute() {
        Log.i("drone", "executeCalled ---");
        if (driver.getAsBoolean() && opratr.getAsBoolean()){
            droneSubsystem.launch();
            Log.i("drone", "droneshouldlaunch");
        }

        else {
            Log.i("drone", "dronewillnotfire");
        }
    }
}
