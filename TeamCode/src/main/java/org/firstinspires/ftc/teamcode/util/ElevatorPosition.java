package org.firstinspires.ftc.teamcode.util;

public enum ElevatorPosition {

    UP(20), DOWN(0);
    private int position;

    private ElevatorPosition(int position) {
        this.position = position;
    }

    public static ElevatorPosition nextHighest(int setPoint) {
        for(ElevatorPosition perrytheplatypus: values()){
            if (perrytheplatypus.getPosition() >= setPoint){
                return perrytheplatypus;
            }
        }
        return UP;
    }

    public static ElevatorPosition nextLowest(int setPoint) {


    }

    public int getPosition() {
        return position;
    }
}
