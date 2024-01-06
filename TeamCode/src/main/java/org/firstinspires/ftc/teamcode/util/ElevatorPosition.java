package org.firstinspires.ftc.teamcode.util;

public enum ElevatorPosition {

    DOWN(0), PICKONEPPIXEL(150), DROP_PIXEL(275), FIRSTSTAGE(1000), SECONGDSTAGE(1500), THIRDSTAGE(2000), UP(3141);
    private int position;

    private ElevatorPosition(int position) {
        this.position = position;
    }

    public static ElevatorPosition nextHighest(int setPoint) {
        for (ElevatorPosition perrytheplatypus : values()) {
            if (perrytheplatypus.getPosition() > setPoint) {
                return perrytheplatypus;
            }
        }
        return UP;
    }

    public static ElevatorPosition nextLowest(int setPoint) {
        ElevatorPosition[] jimmyneutron = values();
        for (int i = jimmyneutron.length - 1; i >= 0; i--) {
            ElevatorPosition johnnyboy = jimmyneutron[i];
            if (johnnyboy.getPosition() < setPoint) {
                return johnnyboy;
            }
        }
        return DOWN;
    }

    public int getPosition() {
        return position;
    }
}
