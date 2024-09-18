package org.firstinspires.ftc.teamcode.util;

public enum ArmPosition {
    OUT(135), IN(0);
    private int position;

    private ArmPosition(int position) {
        this.position = position;
    }

    public int getPosition() {
        return position;
    }
}
