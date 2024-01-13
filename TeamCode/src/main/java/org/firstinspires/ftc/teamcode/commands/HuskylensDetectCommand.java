package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PixelPosition;

import java.util.ArrayList;
import java.util.List;

public class HuskylensDetectCommand extends CommandBase {

    private HuskyLensSubsystem huskyLensSubsystem;
    private List<HuskyLens.Block> lastBlocks;
    private Telemetry telemetry;
    private Alliance alliance;
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private ClawSubsystem clawSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ImuSubsystem imuSubsystem;
    private int count;
    private double positionConfidence = 0.7;

    public HuskylensDetectCommand(HuskyLensSubsystem huskyLensSubsystem, Telemetry telemetry, Alliance alliance, MecanumDriveSubsystem mecanumDriveSubsystem, ClawSubsystem clawSubsystem, ElevatorSubsystem elevatorSubsystem, ImuSubsystem imuSubsystem) {
        this.huskyLensSubsystem = huskyLensSubsystem;
        this.telemetry = telemetry;
        this.alliance = alliance;

        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.imuSubsystem = imuSubsystem;
    }

    @Override
    public void initialize() {

        this.lastBlocks = new ArrayList<>();
        count = 0;

    }

    @Override
    public void execute() {
        List<HuskyLens.Block> detectedBlocks = huskyLensSubsystem.detectBlocks(alliance);
        telemetry.addLine("pixelPosition is" + huskyLensSubsystem.getPixelPosition());
        count++;

        Log.i("huskyBlocks", "count is" + count);
        for (HuskyLens.Block block : detectedBlocks) {
            double blockRatio =  (double) block.height / (double) block.width;
            Log.i("huskyBlocks", "blockRatio" + blockRatio);
            Log.i("huskyBlocks", "block coordinates" + "(" + block.x + "," + block.y + ")");

            if (block.width > 40) {

                lastBlocks.add(block);

            }

        }

    }


    @Override
    public boolean isFinished() {

        if (count >= 15) {

            return true;
        } else {

            return false;

        }

    }

    @Override
    public void end(boolean interrupted) {

        int left = 0;
        int right = 0;
        int center = 0;

        for (HuskyLens.Block block : lastBlocks) {

            if (block.x < 160) {

                left++;

            } else {

                center++;
            }


        }

        right = count - left - center;
        if (alliance == Alliance.RED) {
            int temp = left;
            left = right;
            right = temp;

        }

        Log.i("huskyBlocks", "left, center, right " + left + "," + center + "," + right);

        PixelPosition pixelPosition;

        if (left > right && left > center && left / (double) count > positionConfidence) {

            pixelPosition = PixelPosition.LEFT;

        }

        else if (right > center && right > left && right / (double) count > positionConfidence) {

            pixelPosition = PixelPosition.RIGHT;
        }

        else {

            pixelPosition = PixelPosition.CENTER;
        }

        Command firstPixel = CommandGroups.dropOffFirstPixel(pixelPosition,  alliance,  mecanumDriveSubsystem,  clawSubsystem,  elevatorSubsystem,  telemetry,  imuSubsystem);

        CommandScheduler.getInstance().schedule(firstPixel);
    }



}