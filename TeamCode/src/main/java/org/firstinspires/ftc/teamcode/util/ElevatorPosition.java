package org.firstinspires.ftc.teamcode.util;

public enum ElevatorPosition {

    DOWN(0),
    INTAKE(105),
    UPTHING(2180), UPPERBASKET(6010);
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
        return UPPERBASKET;
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
