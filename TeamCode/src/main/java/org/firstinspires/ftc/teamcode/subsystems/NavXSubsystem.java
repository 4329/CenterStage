package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class NavXSubsystem extends SubsystemBase {
    private IntegratingGyroscope gyro;
    private NavxMicroNavigationSensor navxMicro;
    private Telemetry telemetry;

    public NavXSubsystem(HardwareMap hm, Telemetry telemetry){
        navxMicro = hm.get(NavxMicroNavigationSensor.class,"navx");
        gyro = (IntegratingGyroscope)navxMicro;
        this.telemetry= telemetry;
        navxMicro.
    }

    public double getHeading(){
        double heading=gyro.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return heading;
    }

    public boolean IsCalibrated(){
        return !navxMicro.isCalibrating();
    }

    @Override
    public void periodic() {
        telemetry.addData("NavX Heading","%.2f Deg",getHeading());
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("NavX Heading %.2f", getHeading());
    }
}
