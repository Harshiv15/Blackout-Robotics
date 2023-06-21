package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

// Subsystem for differential arm

@Config
public class ArmSubsystem extends SubsystemBase {
    private final ServoEx leftArm, rightArm;

    // * can be tuned in dashboard
    public static double frontVal = 0.7;
    public static double backVal = 0;
    public static double idleVal = 0.35;

    public ArmSubsystem(ServoEx leftArm, ServoEx rightArm) {
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        leftArm.setInverted(true);
    }

    public Command front() {
        return new InstantCommand(()-> {
            leftArm.setPosition(frontVal);
            rightArm.setPosition(frontVal);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

    public Command back() {
        return new InstantCommand(()-> {
            leftArm.setPosition(backVal);
            rightArm.setPosition(backVal);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

    public Command idle() {
        return new InstantCommand(()-> {
            leftArm.setPosition(idleVal);
            rightArm.setPosition(idleVal);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

    public Command moveWrist(DoubleSupplier power) {
        return new RunCommand(()->{

        }, this);
    }
}
