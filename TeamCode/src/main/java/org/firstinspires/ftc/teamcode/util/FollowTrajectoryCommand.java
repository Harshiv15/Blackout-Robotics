package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class FollowTrajectoryCommand extends CommandBase {
    private final SampleMecanumDrive drive;
    private final Trajectory trajectory;
    private final boolean reversed;

    public FollowTrajectoryCommand(SampleMecanumDrive drive, Trajectory trajectory, boolean reversed) {
        this.drive = drive;
        this.trajectory = trajectory;
        this.reversed = reversed;
    }

    @Override
    public void initialize() {
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}
