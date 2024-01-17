package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;
import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.slf4j.helpers.MessageFormatter;

public class ArmSubsystem extends SubsystemBase {
    private final double MOTOR_SET_POWER=.25;

    private Motor armMotor;
    private Telemetry telemetry;
    private int setPoint;

    public ArmSubsystem(HardwareMap hm, Telemetry jerry) {
        this.armMotor = new Motor(hm, "armMotor");
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
        Log.i(this.getName(), "goToPosition: "+ timmy.getPosition());
        this.setPoint = timmy.getPosition();

        this.armMotor.setTargetPosition(this.setPoint);

    }
    public int getPosition(){
        return armMotor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        this.armMotor.set(MOTOR_SET_POWER);
        telemetry.addLine("Arm actual position: " + armMotor.getCurrentPosition());
        telemetry.addLine("Arm setPoint: " + setPoint);
        telemetry.addLine("armError: " + Math.abs(setPoint - armMotor.getCurrentPosition()));
        Log.i(this.getName(), "periodic: " + this.toString());
    }

    public boolean armAtPosition() {

        return armMotor.atTargetPosition();
    }

    public void stop() {
        this.armMotor.stopMotor();
    }

    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("ArmSubsystem: currentPosition(%d), setPoint(%d)",
                this.getPosition(),
                this.setPoint);
    }

}