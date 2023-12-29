package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class ElevatorSubsystem extends SubsystemBase {
    private int setPoint;
    private Motor elevatorMotor;
    private Telemetry telemetry;
    private TouchSensor elevatorSensor;
    private boolean positionControl = true;

    private static final double DISTANCEPERPULSE = 0.009335691828994;

    public ElevatorSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.telemetry = telemetry;
        elevatorMotor = new Motor(hm, "elevatorMotor");
        this.elevatorSensor = hm.get(TouchSensor.class, "elevatorSensor");
        switchPosition();
    }

    public void goToPosition(ElevatorPosition sammy) {
        this.setPoint = sammy.getPosition();

        this.elevatorMotor.setTargetPosition(this.setPoint);




    }


    @Override
    public void periodic() {
        if (elevatorSensor != null) {
            telemetry.addLine("the elevator is pressed?" + elevatorSensor.isPressed());
        }

        if(this.elevatorMotor!=null && positionControl){
           this.elevatorMotor.set(0.5);

        }

        telemetry.addLine("Elevator setPoint:" + setPoint);
        telemetry.addLine("Eleveator is at:" + elevatorMotor.getCurrentPosition());

    }




    public void stop() {

        this.elevatorMotor.stopMotor();
    }

    public void move(double stickValue) {

//        setPoint += (stickValue * 5.0);

        int newSetPoint = (int) (setPoint + (stickValue * 37.0));

        if (newSetPoint < 0) {

            newSetPoint = 0;

        }

        setPoint = newSetPoint;

        this.elevatorMotor.setTargetPosition(setPoint);

    }

    public void levelUp() {

        ElevatorPosition up = ElevatorPosition.nextHighest(setPoint);
        goToPosition(up);
    }

    public void levelDown() {

        ElevatorPosition down = ElevatorPosition.nextLowest(setPoint);
        goToPosition(down);


    }

    public boolean uThereYet(){

        return elevatorMotor.atTargetPosition();

        }

    public boolean isDown() {
        telemetry.addLine("ELEVATORISDOWN" + elevatorSensor.isPressed());
        return  elevatorSensor.isPressed();
    }

    public void goDown() {
        elevatorMotor.set(-0.5);
    }

    public void goUp() {

        elevatorMotor.set(0.2);

    }

    public void switchPower() {
        this.elevatorMotor.setRunMode(Motor.RunMode.RawPower);
        this.positionControl = false;
    }

    public void switchPosition() {

        elevatorMotor.setDistancePerPulse(DISTANCEPERPULSE);
        this.elevatorMotor.setInverted(true);
        this.elevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        this.elevatorMotor.setPositionCoefficient(1);
        this.elevatorMotor.setFeedforwardCoefficients(0,0.35);
        this.elevatorMotor.setPositionTolerance(0.5);
        this.elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.setPoint = 0;
        this.elevatorMotor.encoder.reset();
        this.positionControl = true;
    }


}