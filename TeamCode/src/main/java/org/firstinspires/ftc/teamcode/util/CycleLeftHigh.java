package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class CycleLeftHigh extends SequentialCommandGroup{
    public CycleLeftHigh(SampleMecanumDrive drive, ElevatorSubsystem elev, ArmSubsystem arm, ClawSubsystem claw, int coneHeight) {
        addCommands(
                new ParallelCommandGroup(
                        new DownSequenceWithPosition(elev, arm, claw, coneHeight),
                        new DelayedCommand(new FollowLeftStackTrajectory(drive), 200)
                ),

                new DelayedCommand(claw.grab(),0).andThen(new DelayedCommand(elev.goTo(Height.HIGH).alongWith(new DelayedCommand(arm.back(), 300)), 100)),

                new DelayedCommand(new FollowLeftHighTrajectory(drive), 150),

                new DelayedCommand(elev.goTo(Height.MEDIUM), 200),
                new DelayedCommand(claw.release(), 250),
                new DelayedCommand(elev.goTo(Height.HIGH), 0)
        );

        addRequirements(elev, arm, claw);
    }
}
