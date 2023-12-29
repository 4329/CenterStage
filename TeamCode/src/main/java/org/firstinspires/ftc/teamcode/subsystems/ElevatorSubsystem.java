package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
    private int setPoint;
    private Motor elevatorMotor;
    private Telemetry telemetry;

    private static final double DISTANCEPERPULSE = 0.009335691828994;

    public ElevatorSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.telemetry = telemetry;
        elevatorMotor = new Motor(hm, "elevatorMotor");
        elevatorMotor.setDistancePerPulse(DISTANCEPERPULSE);
        this.elevatorMotor.setInverted(true);
        this.elevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        this.elevatorMotor.setPositionCoefficient(1);
        this.elevatorMotor.setFeedforwardCoefficients(0,0.35);
        this.elevatorMotor.setPositionTolerance(0.5);
        this.elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.setPoint = 0;
        this.elevatorMotor.encoder.reset();

    }

    public void goToPosition(ElevatorPosition sammy) {
        this.setPoint = sammy.getPosition();

        this.elevatorMotor.setTargetPosition(this.setPoint);

        telemetry.addLine("Elevator setPoint:" + setPoint);
        telemetry.addLine("Eleveator is at:" + elevatorMotor.getCurrentPosition());


    }


    @Override
    public void periodic() {
        if(this.elevatorMotor!=null){
        this.elevatorMotor.set(0.5);
        }
    }

    public void stop() {

        this.elevatorMotor.stopMotor();
    }

    public void move(double stickValue, boolean reset) {

//        setPoint += (stickValue * 5.0);

        int newSetPoint = (int) (setPoint + (stickValue * 37.0));

        if (newSetPoint < 0 && !reset) {

            newSetPoint = 0;

        }

        setPoint = newSetPoint;

        this.elevatorMotor.setTargetPosition(setPoint);

        telemetry.addLine("Elevator setPoint:" + setPoint);

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
        public void reset(){
            this.elevatorMotor.encoder.reset();
        }
}