package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveSubsystem extends SubsystemBase {

    private Telemetry telemetry;
    private MecanumDrive mecanumDrive;
    private Motor leftBackDrive;
    private Motor rightBackDrive;
    private Motor rightFrontDrive;
    private Motor leftFrontDrive;
    private PIDController turnPID;


    public MecanumDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftFrontDrive = new Motor(hardwareMap, "LeftFrontDrive");
        rightFrontDrive = new Motor(hardwareMap, "RightFrontDrive");
        leftBackDrive = new Motor(hardwareMap, "LeftBackDrive");
        rightBackDrive = new Motor(hardwareMap, "RightBackDrive");

        leftFrontDrive.motor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.motor.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.motor.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.motor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mecanumDrive = new MecanumDrive(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        turnPID = new PIDController(1,0,0);
        turnPID.setTolerance(15);
    }

    public void stop() {
        mecanumDrive.stop();
    }

    public Motor.Encoder getSingleEncoder() {
        return leftBackDrive.encoder;
    }

    public void drive(double forward, double turn, double strafe){
        mecanumDrive.driveRobotCentric(-strafe, forward, -turn, false);
    }




}