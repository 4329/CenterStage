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

public class HuskyLensSubsystem extends SubsystemBase {

    private HuskyLens huskyLens;

    private Telemetry telemetry;
    private HuskyLens.Block lastBlock;

    public HuskyLensSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        this.telemetry = telemetry;
        telemetry.update();

    }

    public HuskyLens.Block detectBlocks(Alliance alliance) {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
        }
        if (blocks.length > 0) {

            for (int i = 0; i < blocks.length; i++) {
                if (alliance == Alliance.BLUE && blocks[i].id == 1) {
                    lastBlock = blocks[i];
                    return lastBlock;

                } else if (alliance == Alliance.RED && blocks[i].id == 2) {

                    lastBlock = blocks[i];
                    return lastBlock;
                }

            }

        }

        lastBlock = null;
        return null;

    }

    public PixelPosition getPixelPosition() {

        this.telemetry = telemetry;
        if (lastBlock == null) {
            return PixelPosition.UNKNOWN;
        } else if (lastBlock.x < 160) {
            return PixelPosition.LEFT;

        } else if (lastBlock.x > 161) {
            return PixelPosition.CENTER;
        } else {
            return PixelPosition.RIGHT;
        }
    }

}
