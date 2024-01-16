package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.SpikeMark;
import org.firstinspires.ftc.teamcode.util.SpikeMarkLocation;

import java.util.ArrayList;
import java.util.List;

public class HuskylensDetectCommand extends CommandBase {

    private HuskyLensSubsystem huskyLensSubsystem;
    private List<HuskyLens.Block> lastBlocks;
    private Telemetry telemetry;
    private Alliance alliance;
    private int count;
    private double positionConfidence = 0.7;

    public HuskylensDetectCommand(HuskyLensSubsystem huskyLensSubsystem, Telemetry telemetry, Alliance alliance) {
        this.huskyLensSubsystem = huskyLensSubsystem;
        this.telemetry = telemetry;
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        this.lastBlocks = new ArrayList<>();
        count = 0;
    }

    @Override
    public void execute() {
        List<HuskyLens.Block> detectedBlocks = huskyLensSubsystem.detectBlocks(alliance);
        count++;

        Log.i("huskyBlocks", "count is" + count);
        for (HuskyLens.Block block : detectedBlocks) {
            double blockRatio =  (double) block.height / (double) block.width;
            Log.i("huskyBlocks", "block -> " + block);
            Log.i("huskyBlocks", "blockRatio" + blockRatio);

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
        huskyLensSubsystem.savePicture();

        int spikeOne = 0;
        int spikeThree = 0;
        int spikeTwo = 0;

        for (HuskyLens.Block block : lastBlocks) {
            if (block.x < 160) {
                spikeOne++;
            } else {
                spikeTwo++;
            }
        }

        spikeThree = count - spikeOne - spikeTwo;
        if (alliance == Alliance.RED) {
            int temp = spikeOne;
            spikeOne = spikeThree;
            spikeThree = temp;
        }

        Log.i("huskyBlocks", "spike1, spike2, spike3 " + spikeOne + "," + spikeTwo + "," + spikeThree);

        SpikeMark spikeMark;
        if (spikeOne > spikeThree && spikeOne > spikeTwo && spikeOne / (double) count > positionConfidence) {
            spikeMark = SpikeMark.ONE;
        }
        else if (spikeThree > spikeTwo && spikeThree > spikeOne && spikeThree / (double) count > positionConfidence) {
            spikeMark = SpikeMark.THREE;
        }
        else {
            spikeMark = SpikeMark.TWO;
        }

        SpikeMarkLocation.setCurrentSpikeMark(spikeMark);
    }
}