package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.util.CycleLeftHigh;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.DownSequenceWithPosition;
import org.firstinspires.ftc.teamcode.util.FollowLeftPreloadTrajectory;
import org.firstinspires.ftc.teamcode.util.FollowParkTrajectory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BlackoutAutoLeft extends BaseOpMode {
    @Override
    public void initialize() {

        super.initialize();

        camera.setPipeline(aprilTagDetectionPipeline);

        try {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        }

        while (opModeInInit())
            detectSleeve();

        imu.reset();
        rrDrive.setPoseEstimate(new Pose2d(-35, -63, Math.toRadians(90)));
        schedule(
                new SequentialCommandGroup(

                        claw.grab(),
                        new DelayedCommand(
                            new ParallelCommandGroup(
                                    new FollowLeftPreloadTrajectory(rrDrive),
                                    elev.goTo(Height.HIGH),
                                    arm.idle()
                            ),
                        500),
                        new DelayedCommand(arm.back(), 100),
                        new DelayedCommand(elev.goTo(Height.MEDIUM), 200),
                        new DelayedCommand(claw.release(), 250),
                        new DelayedCommand(elev.goTo(Height.HIGH), 0),

                        new CycleLeftHigh(rrDrive, elev, arm, claw, Height.FIRST.getHeight()),
                        //new CycleLeftHigh(rrDrive, elev, arm, claw, Height.SECOND.getHeight()),
                        //new CycleLeftHigh(rrDrive, elev, arm, claw, Height.THIRD.getHeight()),
                        //new CycleLeftHigh(rrDrive, elev, arm, claw, Height.FOURTH.getHeight()),
                        //new CycleLeftHigh(rrDrive, elev, arm, claw, Height.FIFTH.getHeight()),

                        new ParallelCommandGroup(
                                new FollowParkTrajectory(rrDrive, tagOfInterest),
                                new DelayedCommand(
                                        new DownSequenceWithPosition(elev, arm, claw, Height.GROUND.getHeight()),
                                        100
                                )
                        )

                )
        );

    }
}
