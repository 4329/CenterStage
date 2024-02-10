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
    protected List<HuskyLens.Block> lastBlocks = new ArrayList<>();
    private Telemetry telemetry;
    private Alliance alliance;
    protected int count = 0;
    private double positionConfidence = 0.7;
    protected List<Double> ratios = new ArrayList<>();

    public HuskylensDetectCommand(HuskyLensSubsystem huskyLensSubsystem, Telemetry telemetry, Alliance alliance) {
        this.huskyLensSubsystem = huskyLensSubsystem;
        this.telemetry = telemetry;
        this.alliance = alliance;
        addRequirements(huskyLensSubsystem);
    }

    @Override
    public void initialize() {
        this.lastBlocks = new ArrayList<>();
        count = 0;
        ratios.clear();
    }

    @Override
    public void execute() {

        double maxY = Alliance.BLUE.equals(alliance) ? 195 : 185;

        List<HuskyLens.Block> detectedBlocks = huskyLensSubsystem.detectBlocks(alliance);
        count++;

        Log.i("huskyBlocks", "count is" + count);
        for (HuskyLens.Block block : detectedBlocks) {
            double blockRatio =  (double) block.height / (double) block.width;
            Log.i("huskyBlocks", "block -> " + block);
            Log.i("huskyBlocks", "blockRatio: " + blockRatio);
            huskyLensSubsystem.addTelemetryMessage("block[" + count + "] -> " + block);

            if (block.width > 30 && block.height> 14 && block.y < maxY) {
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
        summary();
    }

    public void summary() {
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
//        if (alliance == Alliance.RED) {
//            int temp = spikeOne;
//            spikeOne = spikeThree;
//            spikeThree = temp;
//        }

        Log.i("huskyBlocks", "spike1, spike2, spike3 " + spikeOne + "," + spikeTwo + "," + spikeThree);
        huskyLensSubsystem.addTelemetryMessage("spike1, spike2, spike3 " + spikeOne + ", " + spikeTwo + ", " + spikeThree);

        double oneConf = spikeOne / (double) count;
        double twoConf = spikeTwo / (double) count;
        double threeConf = spikeThree / (double) count;

        Log.i("huskyBlocks", "confidence (1,2,3): " + oneConf + ", " + twoConf + ", " + threeConf);
        huskyLensSubsystem.addTelemetryMessage("confidence (1,2,3): " + oneConf + ", " + twoConf + ", " + threeConf);

        SpikeMark spikeMark;
        if (spikeOne > spikeThree && spikeOne > spikeTwo && spikeOne / (double) count > positionConfidence) {
            huskyLensSubsystem.addTelemetryMessage("SPIKE MARK: ONE");
            spikeMark = SpikeMark.ONE;
        }
        else if (spikeThree > spikeTwo && spikeThree > spikeOne && spikeThree / (double) count > positionConfidence) {
            huskyLensSubsystem.addTelemetryMessage("SPIKE MARK: THREE");
            spikeMark = SpikeMark.THREE;
        }
        else {
            huskyLensSubsystem.addTelemetryMessage("SPIKE MARK: TWO");
            spikeMark = SpikeMark.TWO;
        }

        SpikeMarkLocation.setCurrentSpikeMark(spikeMark);
    }
}