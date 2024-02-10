package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;
import java.util.List;


public class WebcamSubsystem extends SubsystemBase {

    private VisionPortal visionPortal;
    private AprilTagDetection aprilTagDetection;
    private AprilTagProcessor aprilTagProcessor;
    private Telemetry telemetry;

    private boolean streaming = false;
    private boolean ready = false;


    public WebcamSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        this.visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();


    }


    private void init() {

        if (!streaming) {


            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                streaming = true;
            }

        }


         else {

            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);

            }

            exposureControl.setExposure((long) 6, TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            gainControl.setGain(250);

            ready = true;

        }

    }

    @Override
    public void periodic() {

        if (!ready) {

            init();

        }

    }

    public List<AprilTagDetection> detectTags() {

        return aprilTagProcessor.getDetections();


    }

}

