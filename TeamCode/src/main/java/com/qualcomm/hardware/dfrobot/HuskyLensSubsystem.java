package com.qualcomm.hardware.dfrobot;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.ArrayList;
import java.util.List;

public class HuskyLensSubsystem extends SubsystemBase {

    private HuskyLens huskyLens;

    private Telemetry telemetry;
    private HuskyLens.Block lastBlock;

    private List<String> telemetryMessages = new ArrayList<>();

    public HuskyLensSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        this.telemetry = telemetry;
        telemetry.update();

    }

    public List<HuskyLens.Block> detectBlocks(Alliance alliance) {
        List <HuskyLens.Block> blockList = new ArrayList<>();
        HuskyLens.Block[] blocks = huskyLens.blocks();
        Log.i("huskyBlocks", "Block count: " + blocks.length);

        if (blocks.length > 0) {
            for (int i = 0; i < blocks.length; i++) {
                Log.i("huskyBlocks", "Block[" + i + "]: " + blocks[i].toString());
                if (alliance == Alliance.BLUE && blocks[i].id == 1) {
                    lastBlock = blocks[i];
                    blockList.add(blocks[i]);

                } else if (alliance == Alliance.RED && blocks[i].id == 2) {

                    lastBlock = blocks[i];
                    blockList.add(blocks[i]);
                }
            }
        }

        return blockList;
    }

    public void savePicture() {
        huskyLens.sendCommand((byte)0x39);
    }

    public void clearMessages() {
        telemetryMessages.clear();
    }

    public void addTelemetryMessage(String msg) {
        telemetryMessages.add(msg);
    }

    @Override
    public void periodic() {
        for (String msg : telemetryMessages) {
            telemetry.addData(msg, "");
        }
    }
}
