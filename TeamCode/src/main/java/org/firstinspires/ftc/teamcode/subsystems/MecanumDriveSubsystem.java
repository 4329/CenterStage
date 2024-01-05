package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MecanumDriveSubsystem extends SubsystemBase {

    private Telemetry telemetry;
    private MecanumDrive mecanumDrive;
    private MotorEx leftBackDrive;
    private MotorEx rightBackDrive;
    private MotorEx rightFrontDrive;
    private MotorEx leftFrontDrive;
    private PIDController turnPID;


    public MecanumDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftFrontDrive = new MotorEx(hardwareMap, "LeftFrontDrive");
        rightFrontDrive = new MotorEx(hardwareMap, "RightFrontDrive");
        leftBackDrive = new MotorEx(hardwareMap, "LeftBackDrive");
        rightBackDrive = new MotorEx(hardwareMap, "RightBackDrive");

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

    public enum DriveDegrees {
        RightFortyFive,
        LeftFortyFive,
        RightOneHundred35,
        LeftOneHundred35,
        RightNinety,
        LeftNinety
    }

    public enum TurnDirection{
        Right,
        Left
    }

    @Override
    public void periodic() {

        if (this.leftFrontDrive != null) {
            Log.i("battery", "left front drive current " + leftFrontDrive.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + "mA");
        }
        if (this.rightBackDrive != null) {
            Log.i("battery", "right back drive current " + rightBackDrive.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + "mA");
        }
        if (this.rightFrontDrive != null) {
            Log.i("battery", "right front drive current " + rightFrontDrive.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + "mA");
        }
        if(this.leftBackDrive!=null) {
            Log.i("battery", "left back drive current " + leftBackDrive.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + "mA");
        }
    }
    public void drive(double forward, double turn, double strafe){
        mecanumDrive.driveRobotCentric(-strafe, forward, -turn, false);
    }




}