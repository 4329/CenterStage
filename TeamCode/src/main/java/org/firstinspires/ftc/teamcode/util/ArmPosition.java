package org.firstinspires.ftc.teamcode.util;

public enum ArmPosition {
    OUT(78), IN(-11);
    private int position;

    private ArmPosition(int position) {
        this.position = position;
    }

    public int getPosition() {
        return position;
    }
}
