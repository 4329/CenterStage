package org.firstinspires.ftc.teamcode.util;

public enum SpikeMark {

    ONE, TWO, THREE, UNKNOWN;


    public int aprilTag(Alliance alliance) {

        int allianceBonus = alliance.equals(Alliance.BLUE) ? 0 : 3;

        if (this.equals(ONE)) {


            return 1 + allianceBonus;

        }

        else if (this.equals(TWO)) {

            return 2 + allianceBonus;
        }

        else {

            return 3 + allianceBonus;
        }


    }

}
