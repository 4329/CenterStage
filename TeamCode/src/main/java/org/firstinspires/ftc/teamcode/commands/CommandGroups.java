package org.firstinspires.ftc.teamcode.commands;


import android.provider.ContactsContract;
import android.util.Log;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;
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
    public static Command totalZero(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry) {

        return new ParallelCommandGroup(
                new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DOWN, telemetry),
                new ArmPositionCommand(armSubsystem, ArmPosition.IN));


    }

    public static Command dropOffFirstPixel(PixelPosition pixelPosition, Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry, ImuSubsystem imuSubsystem) {

        double allienceDirection = Alliance.BLUE.equals(alliance) ? 1.0 : -1.0;

        Command turnLeft = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);
        Command turnRight = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90 * allienceDirection);


        Command openclaw = new UnInstantCommand(()-> clawSubsystem.open());
        Command closeclaw = new UnInstantCommand(()-> clawSubsystem.close());
        Command upounopixelo = new ElevatorPosCommand (elevatorSubsystem, ElevatorPosition.PICKONEPPIXEL, telemetry);


        if (alliance == Alliance.BLUE && pixelPosition == PixelPosition.LEFT || (alliance == Alliance.RED && pixelPosition == PixelPosition.RIGHT)) {

            EncoderDriveCommand firstDrive = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 1.5);

            EncoderDriveCommand drive1 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 1.5);
            EncoderDriveCommand drive2 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 9);
            EncoderDriveCommand drive3 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 37);


            EncoderDriveCommand backUp1 = new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0, 3);
            EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0, 6);

            EncoderDriveCommand strafe = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 17);
            EncoderDriveCommand strafe1 = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 10);

            EncoderDriveCommand strafeRightToLeftPixel = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, 0.35  * allienceDirection, 27.5);

            Log.i("huskyBlocks", "pixelPosition " + pixelPosition);

            return new SequentialCommandGroup(firstDrive, turnLeft,
                    new WaitCommand(75),
                    drive1,
                    strafeRightToLeftPixel,
                    openclaw,
                    new WaitCommand(750),
                    upounopixelo,
                    closeclaw,
                    new WaitCommand(400),
                    backUp1,
                    strafe,
                    drive2, //* this is a good spot for april tags *//
                    strafe1,
                    drive3,
                    openclaw,
                    backUp2

            );

        } else if (alliance == Alliance.BLUE && pixelPosition == PixelPosition.RIGHT ||  (alliance == Alliance.RED && pixelPosition == PixelPosition.LEFT)) {

            EncoderDriveCommand strafeLeftToRightPixel = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 20.5);
            EncoderDriveCommand driveforward = new EncoderDriveCommand(mecanumDriveSubsystem, -.35, 0, 0, 7);
            EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 8);
            Command turnAround = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);

            Log.i("huskyBlocks", "pixelPosition " + pixelPosition);

            return new SequentialCommandGroup(turnRight,
                    new WaitCommand(75),
                    strafeLeftToRightPixel,
                    driveforward,
                    openclaw,
                    new WaitCommand(750),
                    upounopixelo,
                    closeclaw,
                    new WaitCommand(400),
                    backUp2,
                    turnAround
                    );

        } else {

            EncoderDriveCommand driveToCenterPosition = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 21);
            EncoderDriveCommand backUp1 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 6);

            Log.i("huskyBlocks", "pixelPosition " +pixelPosition);

            return new SequentialCommandGroup(
                    driveToCenterPosition,
                    openclaw,
                    new WaitCommand(750),
                    upounopixelo,
                    closeclaw,
                    new WaitCommand(400),
                    backUp1,
                    turnLeft
                    );

        }
    }
}




