package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PixelPosition;

public class HuskyLensSubsystem extends SubsystemBase {

    private HuskyLens huskyLens;

    private Telemetry telemetry;
    private HuskyLens.Block lastBlock;

    public HuskyLensSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        this.telemetry = telemetry;
        telemetry.update();

    }

    public HuskyLens.Block detectBlocks() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
        }
        if (blocks.length > 0) {
            lastBlock = blocks[0];
            return blocks[0];
        } else {
            lastBlock = null;
            return null;

        }
    }

    public PixelPosition getPixelPosition() {
        if (lastBlock == null) {
            return PixelPosition.UNKNOWN;
        } else if (lastBlock.x < 80) {
            return PixelPosition.LEFT;
        } else if (lastBlock.x > 240) {
            return PixelPosition.RIGHT;
        } else {
            return PixelPosition.CENTER;
        }
    }


}
