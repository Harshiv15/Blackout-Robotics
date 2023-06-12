package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@Config
public class ClawSubsystem extends SubsystemBase {
    private final Servo clawLeft, clawRight;
    public static double grabPositionLeft = 0.05, grabPositionRight = 0.49;
    public static double releasePositionLeft = 0.6, releasePositionRight = 0;

    public ClawSubsystem(Servo clawLeft, Servo clawRight) {
        this.clawLeft = clawLeft;
        this.clawRight = clawRight;
    }

    public Command grab() {
        return new InstantCommand(() -> {
            clawLeft.setPosition(grabPositionLeft);
            clawRight.setPosition(grabPositionRight);
        }, this);
    }

    public Command release() {
        return new InstantCommand(() -> {
            clawLeft.setPosition(releasePositionLeft);
            clawRight.setPosition(releasePositionRight);
        }, this);
    }
}