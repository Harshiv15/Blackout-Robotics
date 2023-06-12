package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ToGroundCommand extends ParallelCommandGroup {
    public ToGroundCommand(ElevatorSubsystem elev, ArmSubsystem arm) {
        addCommands(
                new ParallelCommandGroup(

                )
        );

        addRequirements(elev, arm);
    }
}
