package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(57.5, 11.5, Math.toRadians(0)), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(57.5, 12.5, 0))
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
