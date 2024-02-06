package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ImuSubsystem extends SubsystemBase {
    private NavxMicroNavigationSensor navxMicro;
    private Telemetry telemetry;

    public ImuSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        this.telemetry = telemetry;
    }

    public void imuTelemetry() {
        Orientation orientation = navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        AngularVelocity angularVelocity = navxMicro.getAngularVelocity(AngleUnit.DEGREES);
        telemetry.addData("Heading", "%.2f Deg.", orientation.firstAngle);
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.thirdAngle);
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.secondAngle);
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
    }

    public boolean isCalibrating() {
        return navxMicro.isCalibrating();
    }


    public double getHeading() {
        Orientation angularOrientation = navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angularOrientation.firstAngle;
    }

    @Override
    public void periodic() {
        imuTelemetry();
    }

    public void reset(){
        // call initialize again??
        //navxMicro.initialize();
    }
}