package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class DownSequenceWithPosition extends SequentialCommandGroup {
    public DownSequenceWithPosition(ElevatorSubsystem elev, ArmSubsystem arm, ClawSubsystem claw, int height) {
        addCommands(
                new ParallelCommandGroup(
                        //arm.front(),
                        new DelayedCommand(elev.goTo(height), 100),
                        new DelayedCommand(claw.release(), 250)
                )
        );

        addRequirements(elev, arm, claw);
    }
}
