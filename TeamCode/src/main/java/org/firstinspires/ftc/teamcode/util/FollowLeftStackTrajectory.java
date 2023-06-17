package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class FollowLeftStackTrajectory extends CommandBase {
    private final SampleMecanumDrive drive;

    public FollowLeftStackTrajectory(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(1)
                .splineTo(new Vector2d(-59, -12), Math.toRadians(180))
                .build()
        );
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
