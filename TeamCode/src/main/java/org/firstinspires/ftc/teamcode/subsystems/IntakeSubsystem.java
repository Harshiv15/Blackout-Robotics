package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final ServoEx claw;
    public static double grabPosition = .5;
    public static double releasePosition = .25;

    public IntakeSubsystem(ServoEx claw) {
        this.claw = claw;
    }

    public Command grab() {
        return new InstantCommand(() -> claw.setPosition(grabPosition), this);
    }

    public Command release() {
        return new InstantCommand(() -> claw.setPosition(releasePosition), this);
    }
}