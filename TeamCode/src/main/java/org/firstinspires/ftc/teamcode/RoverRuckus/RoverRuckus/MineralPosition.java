package org.firstinspires.ftc.teamcode.RoverRuckus.RoverRuckus;

public enum MineralPosition {
    LEFT("Left"),
    CENTER("Center"),
    RIGHT("Right"),
    NOTVISIBLE("None");

    private String str;

    MineralPosition(String str) {
        this.str = str;
    }

    public String getStateVal() {
        return str;
    }

}
