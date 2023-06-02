package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

public class FlipperSubsystem extends SubsystemBase {
    private final ServoEx flipper;
    public static double inPos = 1;
    public static double outPos = 0.61;

    public FlipperSubsystem(ServoEx flipper){
        this.flipper = flipper;
    }

    public Command in() { return new InstantCommand(() -> flipper.setPosition(inPos), this);
    }

    public Command out() {
        return new InstantCommand(() -> flipper.setPosition(outPos), this);
    }


}
