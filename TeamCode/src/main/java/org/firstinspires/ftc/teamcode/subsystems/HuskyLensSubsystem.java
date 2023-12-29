package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TargetPosition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class HuskyLensSubsystem extends SubsystemBase {

    private HuskyLens huskyLens;

    private Telemetry telemetry;
    private HuskyLens.Block lastBlock;

    private final int leftTarget = 80;
    private final int rightTarget = 240;

    public HuskyLensSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        this.telemetry = telemetry;
        // telemetry.update();
    }

    public HuskyLens.Block detectBlocks() {
        List<HuskyLens.Block> blocks = Arrays.asList(huskyLens.blocks());
        telemetry.addData("Block count", blocks.size());
        for (int i = 0; i < blocks.size(); i++) {
            telemetry.addData("Block", blocks.get(i).toString());
        }
        if (blocks.size() > 0) {
            lastBlock = blocks.get(0);
            return blocks.get(0);
        } else {
            lastBlock = null;
            return null;
        }
    }

    public TargetPosition getTargetPosition() {
        if (lastBlock == null) {
            return TargetPosition.UNKNOWN;
        } else if (lastBlock.x < leftTarget) {
            return TargetPosition.LEFT;
        } else if (lastBlock.x > rightTarget) {
            return TargetPosition.RIGHT;
        } else {
            return TargetPosition.CENTER;
        }
    }
}
