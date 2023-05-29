package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.FollowTrajectorySequenceCommand;

@Autonomous
public class BlackoutAutoLeft extends BaseOpMode {
    private TrajectorySequence startToPole, stacktoPole, poleToStack, poleToZone;
    @Override
    public void initialize() {
        super.initialize();
        initTrajectories();
        rrDrive.setPoseEstimate(new Pose2d(36, 64, Math.toRadians(-90)));
        schedule(
                new SequentialCommandGroup(
                        new FollowTrajectorySequenceCommand(rrDrive, startToPole),
                        new FollowTrajectorySequenceCommand(rrDrive, poleToStack),
                        new FollowTrajectorySequenceCommand(rrDrive, stacktoPole),
                        new FollowTrajectorySequenceCommand(rrDrive, poleToStack),
                        new FollowTrajectorySequenceCommand(rrDrive, stacktoPole),
                        new FollowTrajectorySequenceCommand(rrDrive, poleToStack),
                        new FollowTrajectorySequenceCommand(rrDrive, stacktoPole),
                        new FollowTrajectorySequenceCommand(rrDrive, poleToStack),
                        new FollowTrajectorySequenceCommand(rrDrive, stacktoPole),
                        new FollowTrajectorySequenceCommand(rrDrive, poleToStack),
                        new FollowTrajectorySequenceCommand(rrDrive, stacktoPole),
                        new FollowTrajectorySequenceCommand(rrDrive, poleToZone)
                )

        );
    }

    private void initTrajectories() {
        startToPole = rrDrive.trajectorySequenceBuilder(rrDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(33.00, 6, Math.toRadians(35.00)))
                .build();

        stacktoPole = rrDrive.trajectorySequenceBuilder(new Pose2d(57.50, 12.50, Math.toRadians(0.00)))
                .lineToLinearHeading(new Pose2d(33.00, 6, Math.toRadians(35.00)))
                .build();

        // follows stackToPole in reverse because there's no point creating a separate trajectory or command
        poleToStack = rrDrive.trajectorySequenceBuilder(new Pose2d(33.00, 6, Math.toRadians(35.00)))
                .setReversed(true)
                .lineToLinearHeading(stacktoPole.start())
                .build();


        poleToZone = rrDrive.trajectorySequenceBuilder(stacktoPole.end())
                .lineToLinearHeading(new Pose2d(36.00, 30.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(70, 30, Math.toRadians(-180)))
                .build();
    }
}
