package org.firstinspires.ftc.teamcode.commands;


import android.service.carrier.CarrierMessagingService;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;

public class TotalZeroCommandGroup {

public static Command totalZero(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry) {

    return new ParallelCommandGroup (
            new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DOWN, telemetry),
            new UnInstantCommand(() -> clawSubsystem.open()),
            new ArmPositionCommand(armSubsystem, ArmPosition.IN));


}



}



