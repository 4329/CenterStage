package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

public class SpikeMarkLocation {

    private static SpikeMark currentSpikeMark;

    public static SpikeMark getCurrentSpikeMark() {

        Log.i("HuskyBlocks", "spike mark is" + currentSpikeMark);
        return currentSpikeMark;
    }

    public static void setCurrentSpikeMark(SpikeMark currentSpikeMark) {

        SpikeMarkLocation.currentSpikeMark = currentSpikeMark;
    }
}
