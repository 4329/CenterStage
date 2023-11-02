package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem extends SubsystemBase {

    private Telemetry telemetry;
    private Servo servo;
    private boolean closed = true;

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.servo = hardwareMap.get(Servo.class, "clawServo");

    }
    public void open() {
        servo.setPosition(0.5);
        closed = false;
    }
    public void close() {
        servo.setPosition(0);
        closed = true;
    }

    public void toggleClaw() {
        if (closed == true) {
            open();
        }
        else {
            close();
        }
    }
}
