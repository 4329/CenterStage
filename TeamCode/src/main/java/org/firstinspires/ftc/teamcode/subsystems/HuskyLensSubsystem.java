package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PixelPosition;

import java.util.ArrayList;
import java.util.List;

public class HuskyLensSubsystem extends SubsystemBase {

    private HuskyLens huskyLens;

    private Telemetry telemetry;
    private HuskyLens.Block lastBlock;

    private PixelPosition pixelPosition;

    public HuskyLensSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        this.telemetry = telemetry;
        telemetry.update();

    }

    public List<HuskyLens.Block> detectBlocks(Alliance alliance) {
        List <HuskyLens.Block> blockList = new ArrayList<>();
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
        }
        if (blocks.length > 0) {

            for (int i = 0; i < blocks.length; i++) {
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

    public PixelPosition getPixelPosition() {
        return pixelPosition;
    }

    public void setPixelPosition(PixelPosition pixelPosition) {
        this.pixelPosition = pixelPosition;
    }
}
