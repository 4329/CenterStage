package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ElevatorPosCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TimedDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToHeadingCommand;
import org.firstinspires.ftc.teamcode.commands.UnInstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;

import java.util.Base64;

@Autonomous(name = "Simple Blue", group = "1")
public class BlueSimpleAuto extends CommandOpMode {
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private TelemetryUpdateSubsystem telemetryUpdateSubsystem;
    private ImuSubsystem imuSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ClawSubsystem clawSubsystem;
    private TurnToHeadingCommand turn;
    private ArmSubsystem armSubsystem;

    @Override
    public void initialize() {
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
        telemetryUpdateSubsystem = new TelemetryUpdateSubsystem(telemetry);
        imuSubsystem = new ImuSubsystem(hardwareMap, telemetry);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);
        clawSubsystem = new ClawSubsystem(hardwareMap, telemetry);
        armSubsystem = new ArmSubsystem(hardwareMap, telemetry);

        Command closeclaw = new UnInstantCommand(()-> clawSubsystem.close());
        EncoderDriveCommand driveforward = new EncoderDriveCommand(mecanumDriveSubsystem, -.35, 0, 0, 28.5);
        EncoderDriveCommand backUp1 = new EncoderDriveCommand(mecanumDriveSubsystem, .35, 0, 0, 6);

        EncoderDriveCommand driveToCanvas = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 37);
        turn = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90);
        EncoderDriveCommand strafeRight = new EncoderDriveCommand(mecanumDriveSubsystem, 0, 0, -0.35, 20.5);

        Command openclaw = new UnInstantCommand(()-> clawSubsystem.open());
        Command upounopixelo = new ElevatorPosCommand (elevatorSubsystem, ElevatorPosition.INTAKE, telemetry);
        schedule(new SequentialCommandGroup(closeclaw, new WaitCommand(800), driveforward, new WaitCommand(250), openclaw, new WaitCommand(750), upounopixelo, new WaitCommand(250), closeclaw, new WaitCommand(600), backUp1, turn, strafeRight, driveToCanvas, openclaw));
    }
}
