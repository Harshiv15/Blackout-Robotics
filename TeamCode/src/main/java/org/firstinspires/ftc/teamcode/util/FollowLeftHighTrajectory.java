package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class FollowLeftHighTrajectory extends CommandBase {
    private final SampleMecanumDrive drive;

    public FollowLeftHighTrajectory(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                //.splineToLinearHeading(new Pose2d(25, 27, Math.toRadians(125)), Math.toRadians(125))
                //.strafeLeft(10)
                //.back(15)
                .back(1)
                .splineTo(new Vector2d(-27.5, -1.5), Math.toRadians(35))
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
