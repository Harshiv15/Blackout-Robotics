package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ToLowCommand extends SequentialCommandGroup {
    public ToLowCommand(ElevatorSubsystem elev, ArmSubsystem arm, ClawSubsystem claw) {
        addCommands(
                new ParallelCommandGroup()
        );
        addRequirements(elev, arm, claw);
    }
}
