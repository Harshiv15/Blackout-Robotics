package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class TurnCommand extends CommandBase {
    private final SampleMecanumDrive rrDrive;
    private final double rotation;

    public TurnCommand(SampleMecanumDrive rrDrive, double rotation) {
        this.rrDrive = rrDrive;
        this.rotation = rotation;
    }

    @Override
    public void initialize() {
        rrDrive.followTrajectorySequenceAsync(
                rrDrive.trajectorySequenceBuilder(rrDrive.getPoseEstimate())
                        .turn(Math.toRadians(rotation))
                        .build()
        );
    }

    @Override
    public void execute() {
        rrDrive.update();
    }

    @Override
    public boolean isFinished() {
        return !rrDrive.isBusy();
    }
}
