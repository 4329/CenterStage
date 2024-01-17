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
    public static Command totalZero(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry) {

        return new ParallelCommandGroup(
                new ElevatorResetCommand(elevatorSubsystem, telemetry),
                new ArmPositionCommand(armSubsystem, ArmPosition.IN));

    }

    public static Command dropOffFirstPixel(PixelPosition pixelPosition, Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry, ImuSubsystem imuSubsystem, ArmSubsystem armSubsytem) {

        double allienceDirection
                = Alliance.BLUE.equals(alliance) ? 1.0 : -1.0;


        Command turnLeft = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);
        Command turnRight = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90 * allienceDirection);


        Command openclaw = new UnInstantCommand(()-> clawSubsystem.open());
        Command closeclaw = new UnInstantCommand(()-> clawSubsystem.close());
        Command upounopixelo = new ElevatorPosCommand (elevatorSubsystem, ElevatorPosition.PICKONEPPIXEL, telemetry);


        if (pixelPosition == PixelPosition.LEFT) {

            EncoderDriveCommand firstDrive = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 1.5);

            EncoderDriveCommand drive1 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 1.5);
            EncoderDriveCommand drive2 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 9);
            EncoderDriveCommand drive3 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 37);


            EncoderDriveCommand backUp1 = new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0, 3);
            EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0, 6);

            EncoderDriveCommand strafe = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 17);
            EncoderDriveCommand strafe1 = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 10);

            EncoderDriveCommand strafeRightToLeftPixel = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, 0.35  * allienceDirection, 27.5);

            Log.i("huskyBlocks", "pixelPosition 3" + pixelPosition);

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
                    new WaitCommand(1000),
                    backUp2

            );

        } else if (pixelPosition == PixelPosition.RIGHT) {

//            EncoderDriveCommand strafeLeftToRightPixel = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 25);
            EncoderDriveCommand strafe = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 24);

            EncoderDriveCommand driveforward = new EncoderDriveCommand(mecanumDriveSubsystem, -.35, 0, 0, 3.75);
            EncoderDriveCommand firstDrive = new EncoderDriveCommand(mecanumDriveSubsystem, -.35, 0, 0, 24);
            EncoderDriveCommand drive = new EncoderDriveCommand(mecanumDriveSubsystem, -.35, 0, 0, 40);
            EncoderDriveCommand drive1 = new EncoderDriveCommand(mecanumDriveSubsystem, -.35, 0, 0, 40);
            Command arm1= new ArmPositionCommand(armSubsytem, ArmPosition.OUT);
            Command elevator1 = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.SECONGDSTAGE, telemetry);
            Command elevator2 = new ElevatorResetCommand(elevatorSubsystem, telemetry);



//            EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 10);
          EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 5);

            EncoderDriveCommand backUp3 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 6);


            Command turnAround = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);

            Log.i("huskyBlocks", "pixelPosition 1" + pixelPosition);

            return new SequentialCommandGroup(firstDrive, turnRight,
                    new WaitCommand(75),
                    driveforward,
                    openclaw,
                    new WaitCommand(750),
                    upounopixelo,
                    closeclaw,
                    new WaitCommand(400),
                    backUp2,
                    turnAround,
                    new WaitCommand(200),

                    //* this is a good spot for april tags *//
                    strafe,
                    drive,
                    openclaw,
                    new WaitCommand(1000),
                    backUp3

                    );

        } else {

            EncoderDriveCommand driveToCenterPosition = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 26.5);
            EncoderDriveCommand drive1 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 33);

            EncoderDriveCommand backUp1 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 3);
            EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 6);

            EncoderDriveCommand strafe = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 15.5);
            EncoderDriveCommand strafe1 = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 4);

            EncoderDriveCommand backUp4 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 4);

            Command arm1= new ArmPositionCommand(armSubsytem, ArmPosition.OUT);
            Command arm2 = new ArmPositionCommand(armSubsytem, ArmPosition.IN);

            Command elevator1 = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.FIRSTSTAGE, telemetry);

            Command elevator2 = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DOWN, telemetry);

            Log.i("huskyBlocks", "pixelPosition 2" + pixelPosition);

            return new SequentialCommandGroup(
                    driveToCenterPosition,
                    openclaw,
                    new WaitCommand(750),
                    upounopixelo,
                    closeclaw,
                    new WaitCommand(400),
                    backUp1,
                    turnLeft,
                    new WaitCommand(75),
                    arm1.withTimeout(150),
                    elevator1,
                    drive1,
                    openclaw,
                    new WaitCommand(150),
                    backUp4,
                    arm2.withTimeout(150),
                    elevator2



//                    //* this is a good spot for april tags *//
//                    strafe,
//                    drive1,
//                    openclaw,
//                    new WaitCommand(1000),
//                    backUp2
                    );

        }
    }
}




