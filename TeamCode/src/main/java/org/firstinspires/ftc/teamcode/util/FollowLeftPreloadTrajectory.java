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
                //.lineToLinearHeading(new Pose2d(-31.5, 0, Math.toRadians(215.00)))

                //.lineToLinearHeading(new Pose2d(-35, -33, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-25.5, -1.5, Math.toRadians(215)), Math.toRadians(60))
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
