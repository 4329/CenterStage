package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;
import android.util.Log;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;


public class AprilCamCommand extends CommandBase {

    private WebcamSubsystem webcamSubsystem;
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private Telemetry telemetry;

    private int taggy;

    final double SPEED_GAIN  =  0.02  ;
    final double STRAFE_GAIN =  0.015 ;
    final double TURN_GAIN   =  0.01  ;
    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;
    private double DESIRED_DISTANCE = 10;

    private double rangeError;

    public AprilCamCommand(WebcamSubsystem webcamSubsystem, MecanumDriveSubsystem mecanumDriveSubsystem, Telemetry telemetry, int taggy) {


        this.taggy = taggy;
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.webcamSubsystem = webcamSubsystem;
        this.telemetry = telemetry;
        addRequirements(mecanumDriveSubsystem, webcamSubsystem);


    }

    @Override
    public void initialize() {



    }

    @Override
    public void execute() {

        List<AprilTagDetection> detections = webcamSubsystem.detectTags();

        Optional<AprilTagDetection> detection = detections.stream().filter(T -> T.id == taggy).findFirst();


        if (detection.isPresent()) {

            AprilTagDetection aprilTagDetection = detection.get();

            double  rangeError      = (aprilTagDetection.ftcPose.range - DESIRED_DISTANCE);
            Log.i("april", "range error is " + rangeError);
            double  headingError    = aprilTagDetection.ftcPose.bearing;
            Log.i("april", "heading error is" + headingError);
            double  yawError        = aprilTagDetection.ftcPose.yaw;
            Log.i("april", "yaw error is" + yawError);


            double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            mecanumDriveSubsystem.drive(-drive, -turn, -strafe);

        }

        else {

        mecanumDriveSubsystem.stop();

        }
    }

    @Override
    public void end(boolean interrupted) {

        mecanumDriveSubsystem.stop();
        Log.i("april", "ENDEND");


    }

    @Override
    public boolean isFinished() {

        if (rangeError < 0.5) {

            return true;

        }

        else {

            return false;
        }

    }

}
