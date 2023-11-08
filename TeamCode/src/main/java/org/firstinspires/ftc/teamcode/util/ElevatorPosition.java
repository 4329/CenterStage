package org.firstinspires.ftc.teamcode.util;

public enum ElevatorPosition {

    UP(20), DOWN(0);
    private int position;

    private ElevatorPosition(int position) {
        this.position = position;
    }

    public int getPosition() {
        return position;
    }
}
