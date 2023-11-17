package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;

public class DroneSubsystem extends SubsystemBase {

    private Telemetry telemetry;
    private Servo servo;

    private boolean launched = false;


    public DroneSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.servo = hardwareMap.get(Servo.class, "droneServo");
        load();


    }


    @Override
    public void periodic() {
        telemetry.addLine("servo position is:" + servo.getPosition());
    }

    public void load() {
        servo.setPosition(0);
        launched = false;

        telemetry.addLine("drone loaded");

    }

    public void launch() {
        servo.setPosition(0.15);
        launched = true;

        telemetry.addLine("drone launched");

    }
}


