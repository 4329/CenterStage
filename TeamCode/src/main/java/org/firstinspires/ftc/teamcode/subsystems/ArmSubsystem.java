package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArmPosition;

public class ArmSubsystem extends SubsystemBase {
    private Motor armMotor;
    private Telemetry telemetry;
    private int setPoint;

    public ArmSubsystem(HardwareMap hm, Telemetry jerry) {
        this.armMotor = hm.get(Motor.class, "armMotor");
        this.telemetry = jerry;
        this.armMotor.setRunMode(Motor.RunMode.PositionControl);
        this.armMotor.setPositionCoefficient(0.5);
        this.armMotor.setPositionTolerance(3.141592653597);
        this.armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.setPoint = 0;
        this.armMotor.encoder.reset();

    }

    public void goToPosition(ArmPosition timmy) {
        this.setPoint = timmy.getPosition();

        this.armMotor.setTargetPosition(this.setPoint);

        telemetry.addLine("Arm setPoint:" + setPoint);




    }

    @Override
    public void periodic() {
        this.armMotor.set(0.5);
    }

}