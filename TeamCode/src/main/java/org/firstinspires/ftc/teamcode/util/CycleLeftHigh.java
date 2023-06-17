package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class CycleLeftHigh extends SequentialCommandGroup{
    public CycleLeftHigh(SampleMecanumDrive drive, ElevatorSubsystem elev, ArmSubsystem arm, ClawSubsystem claw, int coneHeight) {
        addCommands(
                new ParallelCommandGroup(
                        // new DownSequenceWithPosition(elev, arm, claw, coneHeight),
                        new FollowLeftStackTrajectory(drive)
                ),

                new DelayedCommand(claw.grab()/*.alongWith(new DelayedCommand(arm.back(), 800))*/,0),

                new DelayedCommand(new FollowLeftHighTrajectory(drive)/*.alongWith(elev.goTo(Height.HIGH))*/, 50),

                //new DelayedCommand(claw.release(), 500),
                //new DelayedCommand(claw.grab(), 200),
                //new DelayedCommand(arm.front(), 0),
                new WaitCommand(100)
        );

        addRequirements(elev, arm, claw);
    }
}
