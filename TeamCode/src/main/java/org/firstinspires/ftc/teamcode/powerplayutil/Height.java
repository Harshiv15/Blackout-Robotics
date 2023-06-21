package org.firstinspires.ftc.teamcode.powerplayutil;

public enum Height {
    // junction
    NONE(0),
    GROUND(20),
    LOW(600),
    MEDIUM(1140),
    HIGH(1750),
    // cone stack
    // TODO: tune cone stack heights
    FIRST(280),
    SECOND(220),
    THIRD(160),
    FOURTH(100),
    FIFTH(40);

    // height in encoder ticks
    private final int height;

    Height(int height) {
        this.height = height;
    }

    public int getHeight() {
        return height;
    }
}