package org.firstinspires.ftc.teamcode.commands;


import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
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
import org.firstinspires.ftc.teamcode.util.SpikeMark;
import org.firstinspires.ftc.teamcode.util.SpikeMarkLocation;

import java.util.HashMap;

public class CommandGroups {
    public static Command totalZero(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry) {

        return new ParallelCommandGroup(
                new ElevatorResetCommand(elevatorSubsystem, telemetry),
                new ArmPositionCommand(armSubsystem, ArmPosition.IN));

    }

    public static Command dropOffSpike1(Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry, ImuSubsystem imuSubsystem) {


        Log.i("HuskyBlocks", "BackStage1");
        double allienceDirection = Alliance.BLUE.equals(alliance) ? 1.0 : -1.0;

        EncoderDriveCommand firstDrive = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 1.5);
        EncoderDriveCommand drive1 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 1.5);
        EncoderDriveCommand drive2 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 9);
        EncoderDriveCommand drive3 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 37);

        EncoderDriveCommand backUp1 = new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0, 3);
        EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0, 6);

        EncoderDriveCommand strafe = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 17);
        EncoderDriveCommand strafe1 = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 10);
        EncoderDriveCommand strafeRightToLeftPixel = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, 0.35  * allienceDirection, 27.5);

        Command turnToBack= new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);

        Command openclaw = new UnInstantCommand(()-> clawSubsystem.open());
        Command closeclaw = new UnInstantCommand(()-> clawSubsystem.close());
        Command upounopixelo = new ElevatorPosCommand (elevatorSubsystem, ElevatorPosition.PICKONEPPIXEL, telemetry);


        return new SequentialCommandGroup(firstDrive,
                turnToBack,
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
                backUp2);
    }

    public static Command dropOffSpike2(Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry, ImuSubsystem imuSubsystem, ArmSubsystem armSubsystem) {

        double allienceDirection = Alliance.BLUE.equals(alliance) ? 1.0 : -1.0;

        Log.i("HuskyBlocks", "BackStage2");

            Command openclaw = new UnInstantCommand(()-> clawSubsystem.open());
            Command closeclaw = new UnInstantCommand(()-> clawSubsystem.close());
            Command upounopixelo = new ElevatorPosCommand (elevatorSubsystem, ElevatorPosition.PICKONEPPIXEL, telemetry);
            EncoderDriveCommand driveToCenterPosition = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 26.5);
            EncoderDriveCommand drive1 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 33);

            EncoderDriveCommand backUp1 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 3);
            EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 6);

            EncoderDriveCommand strafe = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 15.5);
            EncoderDriveCommand strafe1 = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35  * allienceDirection, 4);

            EncoderDriveCommand backUp4 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 4);

            Command turnLeft = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);

            Command arm1= new ArmPositionCommand(armSubsystem, ArmPosition.OUT);
            Command arm2 = new ArmPositionCommand(armSubsystem, ArmPosition.IN);

            Command elevator1 = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.FIRSTSTAGE, telemetry);
            Command elevator2 = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DOWN, telemetry);



            return new SequentialCommandGroup(driveToCenterPosition,
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


            );
    }

    public static Command dropOffSpike3(Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry, ImuSubsystem imuSubsystem) {

        double allienceDirection = Alliance.BLUE.equals(alliance) ? 1.0 : -1.0;

        Log.i("HuskyBlocks", "BackStage3");


        EncoderDriveCommand strafe = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35 * allienceDirection, 24);
        EncoderDriveCommand driveforward = new EncoderDriveCommand(mecanumDriveSubsystem, -.35, 0, 0, 3.75);
        EncoderDriveCommand firstDrive = new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem, -.35, 0, 0, 24);
        EncoderDriveCommand drive = new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem, -.35, 0, 0, 40);


        EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 10);
        EncoderDriveCommand backUp3 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 6);

        Command turnAround = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);
        Command turnRight = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90 * allienceDirection);

        Command openclaw = new UnInstantCommand(() -> clawSubsystem.open());
        Command closeclaw = new UnInstantCommand(() -> clawSubsystem.close());
        Command upounopixelo = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.PICKONEPPIXEL, telemetry);


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
                new WaitCommand(200),//* this is a good spot for april tags *//
                strafe,
                drive,
                openclaw,
                new WaitCommand(1000),
                backUp3

        );
    }

    public static Command driveToDesiredSpikeMark(Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry, ImuSubsystem imuSubsystem, ArmSubsystem armSubsystem) {
        if (Alliance.BLUE == alliance){
            return new SelectCommand(new HashMap<Object, Command>() {{
                put (SpikeMark.ONE, dropOffSpike1(alliance, mecanumDriveSubsystem, clawSubsystem, elevatorSubsystem, telemetry, imuSubsystem));
                put (SpikeMark.TWO, dropOffSpike2(alliance, mecanumDriveSubsystem, clawSubsystem, elevatorSubsystem, telemetry, imuSubsystem, armSubsystem));
                put (SpikeMark.THREE, dropOffSpike3(alliance, mecanumDriveSubsystem, clawSubsystem, elevatorSubsystem, telemetry, imuSubsystem));
            }},
                    SpikeMarkLocation::getCurrentSpikeMark
            );
        }
        else {
            return new SelectCommand(new HashMap<Object, Command>() {{
                put (SpikeMark.THREE, dropOffSpike1(alliance, mecanumDriveSubsystem, clawSubsystem, elevatorSubsystem, telemetry, imuSubsystem));
                put (SpikeMark.TWO, dropOffSpike2(alliance, mecanumDriveSubsystem, clawSubsystem, elevatorSubsystem, telemetry, imuSubsystem, armSubsystem));
                put (SpikeMark.ONE, dropOffSpike3(alliance, mecanumDriveSubsystem, clawSubsystem, elevatorSubsystem, telemetry, imuSubsystem));
            }},
                    SpikeMarkLocation::getCurrentSpikeMark
            );
        }


    }


    ////////////////////////////// Front Spike 1
    public static Command frontSpike1(Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry, ImuSubsystem imuSubsystem) {
        double allienceDirection = Alliance.BLUE.equals(alliance) ? 1.0 : -1.0;

        Log.i("HuskyBlocks", "FrontStage1");

        EncoderDriveCommand driveTowardBack = new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem, 90,-.35, 0, 0, 3.75);
        EncoderDriveCommand firstDrive = new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem,
                0,-.25, 0, 0, 25);

        EncoderDriveCommand backUp2 = new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem,
                90,.25, 0, 0, 3);

        Command turnToBack = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry,
                90 * allienceDirection);

        Command openclaw = new UnInstantCommand(clawSubsystem::open);

        Command elevateFirstStage = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.FIRSTSTAGE, telemetry);
        Command elevatorDown = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DOWN, telemetry);

        Command strafeToRow3 = new EncoderDriveCommand(mecanumDriveSubsystem,
                imuSubsystem,90,0, 0, .4, 28);
        Command driveToBackStage = new EncoderDriveCommand(mecanumDriveSubsystem,
                imuSubsystem,90,-.45, 0, 0, 66);
        Command strafeBackDrop = new EncoderDriveCommand(mecanumDriveSubsystem,
                imuSubsystem,90,0, 0, -.4, 28);

        return new SequentialCommandGroup(
                // Strafe toward center of spike square
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem, 0,0, 0, -.25*allienceDirection, 2),
                // Drive to spike square
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem, 0,-.25, 0, 0, 26),
                //turn to spike 1
                new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90),
                // Drive to spike
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem, 90,-.25, 0, 0, 3.5),
                // Drop off one pixel
                openclaw,
                new WaitCommand(750),
                new ElevatorPosCommand(elevatorSubsystem,ElevatorPosition.PICKONEPPIXEL,telemetry),
                new UnInstantCommand(clawSubsystem::close),
                new WaitCommand(750),
                // back away from scored pixel
                new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, 90,.25, 0, 0, 3.5),
                // Strafe to row 3
                new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem,90,0, 0, .4, 28),
                // Turn to backstage
                new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90*allienceDirection),
                // Drive to backstage
                new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem,90* allienceDirection,-.55, 0, 0, 54),
                // Rotate to backdro
                new TurnToHeadingCommand(mecanumDriveSubsystem,imuSubsystem,telemetry,128*allienceDirection),
                // Stafe to front of backdrop
                // new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem,90*allienceDirection,0, 0, -.4, 28),
                new WaitCommand(10)
        );
    }

    ///////////////////// Front spike 2
    public static Command frontSpike2(Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry, ImuSubsystem imuSubsystem) {

        double allienceDirection = Alliance.BLUE.equals(alliance) ? 1.0 : -1.0;

            Command elevator1 = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.FIRSTSTAGE, telemetry);

            Command elevator2 = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DOWN, telemetry);

        Log.i("HuskyBlocks", "FrontStage2");

        Command openclaw = new UnInstantCommand(clawSubsystem::open);
        Command upounopixelo = new ElevatorPosCommand (elevatorSubsystem, ElevatorPosition.FIRSTSTAGE, telemetry);
        Command elevatorDown = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DOWN, telemetry);


        Command turn1 = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);

        EncoderDriveCommand driveToCenterPosition = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 26.5);

        EncoderDriveCommand backUp1 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 6);

        return new SequentialCommandGroup(
                // Strafe to center of square
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem, 0,0, 0, -.25*allienceDirection, 2),
                // Drive past spike 2
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem,0,-.35,0,0,46),
                // spin around
                new TurnToHeadingCommand(mecanumDriveSubsystem,imuSubsystem,telemetry,180),
                // Drive to pixel drop off
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem,180,-.35,0,0,2),
                // place one pixel
                openclaw,
                new WaitCommand(750),
                new ElevatorPosCommand (elevatorSubsystem, ElevatorPosition.PICKONEPPIXEL, telemetry),
                new UnInstantCommand(clawSubsystem::close),
                new WaitCommand(750),
                // Backup to center of row 3 tile
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem,180,.35,0,0,6),
                // Turn to backstage
                new TurnToHeadingCommand(mecanumDriveSubsystem,imuSubsystem,telemetry,90*allienceDirection),
                // Drive to backstage
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem,90*allienceDirection,-.35,0,0,60),
                // Turn to backdrop
                new TurnToHeadingCommand(mecanumDriveSubsystem,imuSubsystem,telemetry,135*allienceDirection),
                ////
                new WaitCommand(1)
                );
    }

    ////  Front spike 3
    public static Command frontSpike3(Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, Telemetry telemetry, ImuSubsystem imuSubsystem) {
        Log.i("HuskyBlocks", "frontStage3");
        double allienceDirection = Alliance.BLUE.equals(alliance) ? 1.0 : -1.0;

        EncoderDriveCommand firstDrive = new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem,-0.35, 0, 0, 2.5);
        EncoderDriveCommand drive1 = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 1.5);

        EncoderDriveCommand backUp1 = new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0, 1.25);
        EncoderDriveCommand backUp2= new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0, 10.5);

        EncoderDriveCommand strafeRightToLeftPixel = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, 0.35  * allienceDirection, 29);
        EncoderDriveCommand strafe1 = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, 0.35  * allienceDirection, 1);


        Command turnRight = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);

        Command turn1 = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90 * allienceDirection);

        Command openclaw = new UnInstantCommand(clawSubsystem::open);
        Command closeclaw = new UnInstantCommand(clawSubsystem::close);
        Command upounopixelo = new ElevatorPosCommand (elevatorSubsystem, ElevatorPosition.FIRSTSTAGE, telemetry);
        Command elevatorDown = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DOWN, telemetry);

        return new SequentialCommandGroup(
                // Strafe to center of square
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem, 0,0, 0, -.25*allienceDirection, 2),
                // Drive to spike square
                new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem,0,-0.35, 0, 0, 25),
                //turn to face front
                new TurnToHeadingCommand(mecanumDriveSubsystem,imuSubsystem,telemetry,-90),
                // Drive to pixel
                new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem,-90,-.35, 0,0, 1.5),
                // Drop off one pixel
                openclaw,
                new WaitCommand(750),
                new ElevatorPosCommand (elevatorSubsystem, ElevatorPosition.PICKONEPPIXEL, telemetry),
                new UnInstantCommand(clawSubsystem::close),
                new WaitCommand(750),
                // backup to center of spike square
                new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem,-90,.35, 0,0, 1.5),
                //strafe to row 3
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem,-90,0,0,-.35,23),
                // turn to backstage
                new TurnToHeadingCommand(mecanumDriveSubsystem,imuSubsystem,telemetry,90*allienceDirection),
                // Drive to backstage
                new EncoderDriveCommand(mecanumDriveSubsystem,imuSubsystem,90*allienceDirection,-.75,0,0,60),
                // Turn to back drop
                new TurnToHeadingCommand(mecanumDriveSubsystem,imuSubsystem,telemetry,135*allienceDirection),
                new WaitCommand(10)
        );
    }

    ///////////////////////////////////////////////////
    public static Command driveToDesiredFrontSpikeMark(Alliance alliance,
                                                       MecanumDriveSubsystem mecanumDriveSubsystem,
                                                       ClawSubsystem clawSubsystem,
                                                       ElevatorSubsystem elevatorSubsystem,
                                                       Telemetry telemetry,
                                                       ImuSubsystem imuSubsystem) {
        return new SelectCommand(new HashMap<Object, Command>() {{
            put(SpikeMark.ONE, frontSpike1(alliance, mecanumDriveSubsystem, clawSubsystem, elevatorSubsystem, telemetry, imuSubsystem));
            put(SpikeMark.TWO, frontSpike2(alliance, mecanumDriveSubsystem, clawSubsystem, elevatorSubsystem, telemetry, imuSubsystem));
            put(SpikeMark.THREE, frontSpike3(alliance, mecanumDriveSubsystem, clawSubsystem, elevatorSubsystem, telemetry, imuSubsystem));
          }},
          SpikeMarkLocation::getCurrentSpikeMark
        );
    }
}
