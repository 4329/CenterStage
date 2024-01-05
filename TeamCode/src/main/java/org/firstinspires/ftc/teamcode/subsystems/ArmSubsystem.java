package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.ArmPosition;

public class ArmSubsystem extends SubsystemBase {
    private MotorEx armMotor;
    private Telemetry telemetry;
    private int setPoint;

    public ArmSubsystem(HardwareMap hm, Telemetry jerry) {
        this.armMotor = new MotorEx(hm, "armMotor");
        this.armMotor.setInverted(false);
        this.telemetry = jerry;
        this.armMotor.setRunMode(Motor.RunMode.PositionControl);
        this.armMotor.setPositionCoefficient(0.15);
        this.armMotor.setPositionTolerance(0.5);
        this.armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.setPoint = 0;
        this.armMotor.encoder.reset();

    }

    public void goToPosition(ArmPosition timmy) {
        this.setPoint = timmy.getPosition();

        this.armMotor.setTargetPosition(this.setPoint);







    }

    @Override
    public void periodic() {
        this.armMotor.set(0.1);
        telemetry.addLine("Arm actual position: " + armMotor.getCurrentPosition());
        telemetry.addLine("Arm setPoint: " + setPoint);
        telemetry.addLine("armError: " + Math.abs(setPoint - armMotor.getCurrentPosition()));
        if(this.armMotor!=null) {
            Log.i("battery", "arm current " + armMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + "mA");
        }
    }

    public boolean armAtPosition() {

        return armMotor.atTargetPosition();
    }

    public void stop() {
        this.armMotor.stopMotor();
    }
}