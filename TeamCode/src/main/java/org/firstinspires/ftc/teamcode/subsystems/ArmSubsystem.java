package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Subsystem for differential arm

public class ArmSubsystem extends SubsystemBase {
    private final DcMotorEx leftArmMotor, rightArmMotor;

    public static int frontVal = 50;
    public static int backVal = 50;

    public ArmSubsystem(DcMotorEx leftArmMotor, DcMotorEx rightArmMotor) {
        this.leftArmMotor = leftArmMotor;
        this.rightArmMotor = rightArmMotor;
    }

    // only need to move one motor in auto
    public Command front() {
        return new InstantCommand(()-> leftArmMotor.setTargetPosition(frontVal));
    }

    public Command back() {
        return new InstantCommand(()-> leftArmMotor.setTargetPosition(backVal));
    }

    // use this for teleop!!
    public Command setAngles(double pitch, double roll) {
        // @TODO - implement setAngles
        return new InstantCommand();
    }
}
