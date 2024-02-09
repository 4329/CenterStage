package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.FrcPidController;

public class EncoderDriveCommand extends CommandBase {
    private MecanumDriveSubsystem drive;
    private ImuSubsystem imuSubsystem;
    private double forward;
    private double turn;
    private double strafe;
    private double inches;
    public static final double TICKS_PER_REV = 28;
    public static final double MAX_RPM = 6000;
    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = .0529100529; // output (wheel) speed / input (motor) speed
    private Motor.Encoder encoder;
    private int startingTicks;

    private double initialHeading;
    private FrcPidController pid;


    public EncoderDriveCommand(MecanumDriveSubsystem mecanumDriveSubsystem,double forward,double turn,double strafe,double inches){
        this(mecanumDriveSubsystem,null,forward,turn,strafe,inches);

    }
    public EncoderDriveCommand(MecanumDriveSubsystem mecanumDriveSubsystem,
                               ImuSubsystem imuSubsystem,
                               double forward, double turn, double strafe, double inches) {
        this(mecanumDriveSubsystem,imuSubsystem,Double.NaN,forward,turn,strafe,inches);
    }

    public EncoderDriveCommand(MecanumDriveSubsystem mecanumDriveSubsystem,
                               ImuSubsystem imuSubsystem,
                               double initialHeading,
                               double forward, double turn, double strafe, double inches) {
        this.drive = mecanumDriveSubsystem;
        this.imuSubsystem= imuSubsystem;
        this.initialHeading= initialHeading;
        this.forward = forward;
        this.turn = turn;
        this.strafe = strafe;
        this.inches = inches;
        this.pid=new FrcPidController(0.04,0,.000);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        encoder = drive.getSingleEncoder();
        startingTicks = encoder.getPosition();
        if (imuSubsystem != null) {
            if(Double.isNaN(this.initialHeading)){
                this.initialHeading = imuSubsystem.getHeading();
            }

            pid.setSetpoint(this.initialHeading);
            pid.setTolerance(.1);
            pid.enableContinuousInput(-180,180);
            Log.i(getName(), String.format("initialize: with IMU - Initial Heading=%.2f",this.initialHeading));
        }
        else {
            Log.i(getName(), "initialize: without IMU");
        }


        Log.i(this.getName(), String.format("initialize: forward(%.2f),turn(%.2f),strafe(%.2f),inches(%.2f)",
                forward,turn,strafe,inches));
    }

    @Override
    public void execute() {
        if(turn!=0 || imuSubsystem == null){
            // If turn non-zero or imuSubsystem is null, then do regular drive command
            Log.i(getName(), String.format( "execute: No IMU with turn = ",turn));
            drive.drive(forward, turn, strafe);
        }
        else {
            // turn is zero, so use PID to keep robot pointing in initial heading
            double currentHeading = imuSubsystem.getHeading();
            double output = pid.calculate(currentHeading);
            output = Range.clip(output,-.25,.25);
            drive.drive(forward, -output, strafe);
            Log.i(this.getName(),
                    String.format("execute: with PID output %.2f for initial heading %.2f and current heading %.2f",
                    -output,
                    initialHeading,
                    currentHeading));
        }

    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        if(imuSubsystem!= null){
            Log.i(getName(), String.format("end: initialHeading=%.2f, ending Heading=%.2f",
                    initialHeading,
                    imuSubsystem.getHeading()));
        }else{
            Log.i(getName(), String.format("end:"));
        }

    }

    @Override
    public boolean isFinished() {
        int larry = Math.abs(encoder.getPosition() - startingTicks);
        double remainingInches = encoderTicksToInches(larry);
        if(remainingInches >= inches){
            return true;
        }
        else{
            return false;
        }
    }
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
