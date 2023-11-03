package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElevatorSubsystem extends SubsystemBase {
    private final double setPoint;
    private Motor elevatorMotor;
    private Telemetry telemetry;

    private static final double DISTANCEPERPULSE = 0.009335691828994;

    public ElevatorSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.telemetry = telemetry;
        elevatorMotor = new Motor(hm, "elevatorMotor");
        elevatorMotor.setDistancePerPulse(DISTANCEPERPULSE);
        this.elevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        this.elevatorMotor.setPositionCoefficient(0.5);
        this.elevatorMotor.setPositionTolerance(0.314159265358979);
        this.elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.setPoint = 0;
        this.elevatorMotor.encoder.reset();

    }

    public void run(double up) {

        elevatorMotor.set(up);
    }

}