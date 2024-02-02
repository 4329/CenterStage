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
        open();


    }


    @Override
    public void periodic() {
        telemetry.addLine("servo position is:" + servo.getPosition());
    }

    public void open() {
        servo.setPosition(0);
        closed = false;

        telemetry.addLine("claw open");

    }
    public void close() {
        servo.setPosition(0.40);
        closed = true;

        telemetry.addLine("claw closed");

    }

    public void onePixel() {
        servo.setPosition(0.30);

        telemetry.addLine("");

        closed = false;

    }

    public void toggleClaw() {
        if (closed == true) {
            open();

            telemetry.addLine("clawOpen");
        }
        else {
            close();

            telemetry.addLine("clawClosed");
        }
    }




}
