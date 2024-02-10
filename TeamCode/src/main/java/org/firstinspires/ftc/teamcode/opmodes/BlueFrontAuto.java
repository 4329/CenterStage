package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.CommandGroups;
import org.firstinspires.ftc.teamcode.commands.ElevatorResetCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderDriveCommand;
import org.firstinspires.ftc.teamcode.commands.HuskylensDetectCommand;
import org.firstinspires.ftc.teamcode.commands.InitializeNavxCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToHeadingCommand;
import org.firstinspires.ftc.teamcode.commands.UnInstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.SpikeMarkLocation;

@Autonomous(name = "BlueAutoFrontStage", group = "1")
public class BlueFrontAuto extends CommandOpMode {
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private TelemetryUpdateSubsystem telemetryUpdateSubsystem;
    private ImuSubsystem imuSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ClawSubsystem clawSubsystem;
    private ArmSubsystem armSubsystem;
    private TurnToHeadingCommand turnToHeadingCommand;
    private HuskyLensSubsystem huskyLensSubsystem;
    private WebcamSubsystem webcamSubsystem;


    @Override
    public void initialize() {

        telemetry.speak("blue blue blue");

        SpikeMarkLocation.setCurrentSpikeMark(null);

        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
        telemetryUpdateSubsystem = new TelemetryUpdateSubsystem(telemetry);
        imuSubsystem = new ImuSubsystem(hardwareMap, telemetry);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);
        huskyLensSubsystem = new HuskyLensSubsystem(hardwareMap, telemetry);
        clawSubsystem = new ClawSubsystem(hardwareMap, telemetry);
        armSubsystem = new ArmSubsystem(hardwareMap, telemetry);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, telemetry);
        Command closeclaw = new UnInstantCommand(()-> clawSubsystem.close());
        Command see = new HuskylensDetectCommand(huskyLensSubsystem,  telemetry, Alliance.BLUE);
        Command elevatorReset = new ElevatorResetCommand(elevatorSubsystem, telemetry);
        Command imuReset = new InitializeNavxCommand(imuSubsystem, telemetry);
        Command scoreTheSpike = CommandGroups.driveToDesiredFrontSpikeMark(Alliance.BLUE, mecanumDriveSubsystem, clawSubsystem, elevatorSubsystem, telemetry, imuSubsystem);
        Command april = CommandGroups.aprilTagScore(Alliance.BLUE, mecanumDriveSubsystem, webcamSubsystem, elevatorSubsystem, armSubsystem, clawSubsystem, telemetry);

        schedule(new SequentialCommandGroup(imuReset.withTimeout(5000), elevatorReset, new WaitCommand(250), closeclaw, see.withTimeout(1500), scoreTheSpike, april));
    }

}
