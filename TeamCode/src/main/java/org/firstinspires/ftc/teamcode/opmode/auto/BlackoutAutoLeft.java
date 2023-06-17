package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.util.CycleLeftHigh;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.FollowLeftHighTrajectory;
import org.firstinspires.ftc.teamcode.util.FollowLeftPreloadTrajectory;
import org.firstinspires.ftc.teamcode.util.FollowLeftStackTrajectory;

@Autonomous
public class BlackoutAutoLeft extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
        imu.reset();
        rrDrive.setPoseEstimate(new Pose2d(-35, -63, Math.toRadians(90)));
        schedule(
                new SequentialCommandGroup(
                        claw.grab(),
                        new ParallelCommandGroup(
                                new DelayedCommand(new FollowLeftPreloadTrajectory(rrDrive), 0)
                                //new DelayedCommand(elev.goTo(Height.HIGH), 150)
                        ),
                        new DelayedCommand(arm.back(), 250),
                        //new DelayedCommand(elev.goTo(Height.MEDIUM), 0),
                        new DelayedCommand(claw.release(), 500),
                        //new DelayedCommand(elev.goTo(Height.HIGH), 250)

                        new CycleLeftHigh(rrDrive, elev, arm, claw, Height.FIRST.getHeight()),
                        new CycleLeftHigh(rrDrive, elev, arm, claw, Height.SECOND.getHeight()),
                        new CycleLeftHigh(rrDrive, elev, arm, claw, Height.THIRD.getHeight()),
                        new CycleLeftHigh(rrDrive, elev, arm, claw, Height.FOURTH.getHeight()),
                        new CycleLeftHigh(rrDrive, elev, arm, claw, Height.FIFTH.getHeight())
                )
        );


    }
}
