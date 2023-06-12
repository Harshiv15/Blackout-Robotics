package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.CycleLeftHigh;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.FollowLeftHighTrajectory;
import org.firstinspires.ftc.teamcode.util.FollowLeftPreloadTrajectory;
import org.firstinspires.ftc.teamcode.util.FollowLeftStackTrajectory;
import org.firstinspires.ftc.teamcode.util.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;

@Autonomous
public class BlackoutAutoLeft extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
        rrDrive.setPoseEstimate(new Pose2d(36, 64, Math.toRadians(-90)));
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                claw.grab(),
                                new DelayedCommand(new FollowLeftPreloadTrajectory(rrDrive), 150),
                                new DelayedCommand(elev.goTo(Height.HIGH), 150)
                                //new DelayedCommand(arm.back(), 150)
                        ),
                        new DelayedCommand(claw.release(), 750),
                        new DelayedCommand(claw.grab(), 200),
                        new CycleLeftHigh(rrDrive, elev, arm, claw, 50),
                        new CycleLeftHigh(rrDrive, elev, arm, claw, 40),
                        new CycleLeftHigh(rrDrive, elev, arm, claw, 30),
                        new CycleLeftHigh(rrDrive, elev, arm, claw, 20),
                        new CycleLeftHigh(rrDrive, elev, arm, claw, 10)
                )
        );


    }
}
