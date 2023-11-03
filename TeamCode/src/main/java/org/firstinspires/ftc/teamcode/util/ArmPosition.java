package org.firstinspires.ftc.teamcode.util;

public enum ArmPosition {OUT(20.3141), IN(0);
    private double position;
    private ArmPosition(double position){
    this.position = position;
    }

    public double getPosition() {
        return position;
    }
}
