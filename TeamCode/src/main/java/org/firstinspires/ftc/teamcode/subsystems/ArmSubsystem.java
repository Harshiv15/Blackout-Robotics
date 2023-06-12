package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;

// Subsystem for differential arm

@Config
public class ArmSubsystem extends SubsystemBase {
    private final Servo leftArm, rightArm;

    public static int frontVal = 1;
    public static int backVal = -1;

    public ArmSubsystem(Servo leftArm, Servo rightArm) {
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        leftArm.setDirection(Servo.Direction.REVERSE);
    }

    public Command front() {
        return new InstantCommand(()-> {
            leftArm.setPosition(frontVal);
            rightArm.setPosition(frontVal);
        }, this);
    }

    public Command back() {
        return new InstantCommand(()-> {
            leftArm.setPosition(backVal);
            rightArm.setPosition(backVal);
        }, this);
    }

    // use this for teleop!!
    public Command setAngles(double pitch, double roll) {
        // @TODO - implement setAngles
        return new InstantCommand();
    }
}
