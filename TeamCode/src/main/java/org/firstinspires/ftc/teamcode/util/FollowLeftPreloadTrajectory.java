package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class FollowLeftPreloadTrajectory extends CommandBase {
    private final SampleMecanumDrive drive;

    public FollowLeftPreloadTrajectory(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                //.lineToLinearHeading(new Pose2d(33.00, 6, Math.toRadians(35.00)))
                .splineToLinearHeading(new Pose2d(36, 19, Math.toRadians(-90)), Math.toRadians(-90))
                .splineTo(new Vector2d(28, 18), Math.toRadians(125))
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
