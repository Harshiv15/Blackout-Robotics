package org.firstinspires.ftc.teamcode.powerplayutil;

public enum Height {
    // junction
    NONE(0),
    GROUND(20),
    LOW(530),
    MEDIUM(1000),
    HIGH(1440),
    // cone stack
    // TODO: tune cone stack heights
    FIRST(-142),
    SECOND(-121),
    THIRD(-87),
    FOURTH(-60);

    // height in encoder ticks
    private final int height;

    Height(int height) {
        this.height = height;
    }

    public int getHeight() {
        return height;
    }
}