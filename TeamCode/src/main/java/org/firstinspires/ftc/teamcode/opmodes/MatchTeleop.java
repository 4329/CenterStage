package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CommandGroups;
import org.firstinspires.ftc.teamcode.commands.ElevatorResetCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorVerticalCommand;
import org.firstinspires.ftc.teamcode.commands.FireZeMisslizCommand;
import org.firstinspires.ftc.teamcode.commands.InitializeNavxCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDpadCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.ArmPosition;

@TeleOp(name = "Match Teleop", group = "1")
public class MatchTeleop extends CommandOpMode {
    // FtcDashboard dashboard = FtcDashboard.getInstance();

    private GamepadEx driver, operator;
    private ElevatorSubsystem elevatorSubsystem;
    private MecanumDriveSubsystem mecanumDriveSubsystem
            ;
     private TelemetryUpdateSubsystem telemetryUpdateSubsystem;
    private ImuSubsystem imuSubsystem;
    private ClawSubsystem clawSubsystem;
    private ArmSubsystem armSubsystem;
    private Command totalZeroCommandGroup;
    private DroneSubsystem droneSubsystem;
    private HuskyLensSubsystem huskyLensSubsystem;
    private WebcamSubsystem webcamSubsystem;
    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
        telemetryUpdateSubsystem = new TelemetryUpdateSubsystem(telemetry);
        imuSubsystem = new ImuSubsystem(hardwareMap, telemetry);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);
        clawSubsystem = new ClawSubsystem(hardwareMap, telemetry);
        armSubsystem = new ArmSubsystem(hardwareMap, telemetry);
        totalZeroCommandGroup = CommandGroups.totalZero(armSubsystem, elevatorSubsystem, telemetry);
        droneSubsystem = new DroneSubsystem(hardwareMap, telemetry);
        huskyLensSubsystem = new HuskyLensSubsystem(hardwareMap, telemetry);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, telemetry);
        FireZeMisslizCommand firedemisillestoaliens = new FireZeMisslizCommand(droneSubsystem,
                () -> driver.getButton(GamepadKeys.Button.RIGHT_BUMPER),
                () -> operator.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        MecanumDriveCommand driveMecanumCommand = new MecanumDriveCommand(mecanumDriveSubsystem,
                () -> -driver.getLeftY(),
                () -> driver.getRightX(),
                () -> driver.getLeftX(),
                () -> driver.getButton(GamepadKeys.Button.LEFT_BUMPER),
                () -> driver.getButton(GamepadKeys.Button.A),
                telemetry);

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileHeld(new MecanumDpadCommand(mecanumDriveSubsystem,() -> driver.getButton(GamepadKeys.Button.B),0, 1, telemetry));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new MecanumDpadCommand(mecanumDriveSubsystem,() -> driver.getButton(GamepadKeys.Button.B),1, 0, telemetry));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new MecanumDpadCommand(mecanumDriveSubsystem,() -> driver.getButton(GamepadKeys.Button.B),-1, 0, telemetry));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(new MecanumDpadCommand(mecanumDriveSubsystem,() -> driver.getButton(GamepadKeys.Button.B),0, -1, telemetry));
        ElevatorVerticalCommand elevatorVerticalCommand = new ElevatorVerticalCommand(elevatorSubsystem,
                () -> operator.getLeftY(),


        telemetry);
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(()-> clawSubsystem.toggleClaw());
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(()-> armSubsystem.goToPosition(ArmPosition.OUT));
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(()-> armSubsystem.goToPosition(ArmPosition.IN));
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(()-> clawSubsystem.onePixel());

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()-> elevatorSubsystem.levelUp());
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()-> elevatorSubsystem.levelDown());
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(totalZeroCommandGroup);
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(()-> clawSubsystem.onePixel());


        mecanumDriveSubsystem.setDefaultCommand(driveMecanumCommand);
        elevatorSubsystem.setDefaultCommand(elevatorVerticalCommand);
        droneSubsystem.setDefaultCommand(firedemisillestoaliens);

        register(imuSubsystem, telemetryUpdateSubsystem);

        schedule(new InitializeNavxCommand(imuSubsystem, telemetry), new ElevatorResetCommand(elevatorSubsystem, telemetry));
    }
}