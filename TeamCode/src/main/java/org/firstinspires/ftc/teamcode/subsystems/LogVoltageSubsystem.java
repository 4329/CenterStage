package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class LogVoltageSubsystem extends SubsystemBase {
    private final VoltageSensor CHsensor;
    private final VoltageSensor EHsensor;

    public LogVoltageSubsystem(HardwareMap HM) {
        this.CHsensor=HM.get(VoltageSensor.class, "Control Hub");
        this.EHsensor=HM.get(VoltageSensor.class, "Expansion Hub");
        }

    @Override
    public void periodic() {
        Log.i("battery", "control hub voltage " + CHsensor.getVoltage());
        Log.i("battery", "expansion hub voltage " + EHsensor.getVoltage());
    }
}
