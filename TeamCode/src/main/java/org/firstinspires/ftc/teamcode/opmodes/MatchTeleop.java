package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.CommandGroups;
import org.firstinspires.ftc.teamcode.commands.ElevatorVerticalCommand;
import org.firstinspires.ftc.teamcode.commands.HuskylensDetectCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToHeadingCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;
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
        totalZeroCommandGroup = CommandGroups.totalZero(armSubsystem, clawSubsystem, elevatorSubsystem, telemetry);
        droneSubsystem = new DroneSubsystem(hardwareMap, telemetry);
        huskyLensSubsystem = new HuskyLensSubsystem(hardwareMap, telemetry);
        MecanumDriveCommand driveMecanumCommand = new MecanumDriveCommand(mecanumDriveSubsystem,
                () -> -driver.getLeftY(),
                () -> driver.getRightX(),
                () -> driver.getLeftX(),
                () -> driver.getButton(GamepadKeys.Button.B),
                telemetry);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(()-> elevatorSubsystem.reset());
        ElevatorVerticalCommand elevatorVerticalCommand = new ElevatorVerticalCommand(elevatorSubsystem,
                () -> operator.getLeftY(),
                () -> driver.getButton(GamepadKeys.Button.RIGHT_BUMPER),

        telemetry);
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(()-> clawSubsystem.toggleClaw());
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(()-> armSubsystem.goToPosition(ArmPosition.OUT));
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(()-> armSubsystem.goToPosition(ArmPosition.IN));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()-> elevatorSubsystem.levelUp());
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()-> elevatorSubsystem.levelDown());
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(totalZeroCommandGroup);
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(()-> droneSubsystem.launch());
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(()-> clawSubsystem.onePixel());

//        operator.getGamepadButton(GamepadKeys.Button.X).whileHeld(new HuskylensDetectCommand(huskyLensSubsystem, telemetry));

        mecanumDriveSubsystem.setDefaultCommand(driveMecanumCommand);
        elevatorSubsystem.setDefaultCommand(elevatorVerticalCommand);
    }
}