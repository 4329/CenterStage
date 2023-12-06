package org.firstinspires.ftc.teamcode.commands;


import android.provider.ContactsContract;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;
import org.firstinspires.ftc.teamcode.util.PixelPosition;
import org.jetbrains.annotations.Contract;

import java.util.Base64;
import java.util.function.Supplier;

import javax.crypto.ExemptionMechanism;

public class CommandGroups {

    @NonNull
    @Contract("_, _, _, _ -> new")
    public static Command totalZero(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry) {

        return new ParallelCommandGroup(
                new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DOWN, telemetry),
                new UnInstantCommand(() -> clawSubsystem.open()),
                new ArmPositionCommand(armSubsystem, ArmPosition.IN));


    }

    public static Command dropOffFirstPixel(Supplier<PixelPosition> pixelPositionSupplier, Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry, ImuSubsystem imuSubsystem) {

        double allienceDirection = Alliance.BLUE.equals(alliance) ? 1.0 : -1.0;
        PixelPosition detectedPixelPosition = pixelPositionSupplier.get();

        Command turnLeft = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);
        Command turnRight = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90 * allienceDirection);
        Command turnAround = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 180 * allienceDirection);

        EncoderDriveCommand strafeRightToLeftPixel = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35, 22);
        EncoderDriveCommand driveToStart = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 8);

        Command openclaw = new UnInstantCommand(()-> clawSubsystem.open());
        Command closeclaw = new UnInstantCommand(()-> clawSubsystem.close());
        Command upounopixelo = new ElevatorPosCommand (elevatorSubsystem, ElevatorPosition.DROP_PIXEL, telemetry);


        if (PixelPosition.LEFT.equals(detectedPixelPosition)) {

            return new SequentialCommandGroup(turnLeft);

        } else if (PixelPosition.RIGHT.equals(detectedPixelPosition)) {

            EncoderDriveCommand strafeLeftToRightPixel = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35, 17);
            EncoderDriveCommand driveforward = new EncoderDriveCommand(mecanumDriveSubsystem, -.35, 0, 0, -4);
            EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 6);


            return new SequentialCommandGroup(turnRight,
                    strafeLeftToRightPixel,
                    driveforward,
                    openclaw,
                    new WaitCommand(750),
                    upounopixelo,
                    closeclaw,
                    backUp2,
                    turnAround
                    );

        } else {

            EncoderDriveCommand driveToCenterPosition = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 21.5);
            EncoderDriveCommand backUp1 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 6);


            return new SequentialCommandGroup(
                    driveToCenterPosition,
                    openclaw,
                    new WaitCommand(750),
                    upounopixelo,
                    closeclaw,
                    new WaitCommand(400), backUp1,
                    turnLeft
                    );

        }
    }
}



