package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.opmode.BaseOpMode.LEFT;
import static org.firstinspires.ftc.teamcode.opmode.BaseOpMode.MIDDLE;
import static org.firstinspires.ftc.teamcode.opmode.BaseOpMode.RIGHT;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

public class FollowParkTrajectory extends CommandBase {
    private final SampleMecanumDrive drive;
    private final AprilTagDetection aprilTagDetection;
    private TrajectorySequence parkTraj;

    public FollowParkTrajectory(SampleMecanumDrive drive, AprilTagDetection aprilTagDetection) {
        this.drive = drive;
        this.aprilTagDetection = aprilTagDetection;
    }

    @Override
    public void initialize() {
        // * default trajectory (right or not sighted or not one of LEFT, MIDDLE, or RIGHT)
        if(aprilTagDetection == null
                || (aprilTagDetection.id != LEFT && aprilTagDetection.id != MIDDLE && aprilTagDetection.id != RIGHT)
                || aprilTagDetection.id == RIGHT) {
            parkTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(1)
                    .splineTo(new Vector2d(-35, -9), Math.toRadians(180))
                    .back(24)
                    .build();
        } else if(aprilTagDetection.id == LEFT) {
            // * left trajectory
            parkTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(1)
                    .splineTo(new Vector2d(-35, -9), Math.toRadians(180))
                    .forward(24)
                    .build();
        } else if(aprilTagDetection.id == MIDDLE) {
            // * middle trajectory
            parkTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(1)
                    .splineTo(new Vector2d(-35, -9), Math.toRadians(180))
                    .build();
        }

        drive.followTrajectorySequenceAsync(parkTraj);
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
