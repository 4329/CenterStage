package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class RoboteerHuskyLens extends HuskyLens {
    public RoboteerHuskyLens(I2cDeviceSynch deviceClient) {
        super(deviceClient);
    }

    public void savePictureOnSdCard() {
        this.sendCommand((byte) 0x30);
        // that's probably sufficient;
    }
}
