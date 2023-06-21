package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.*;

import static org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem.pix_to_degree;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Config
@TeleOp
public class AlignTesting extends BaseOpMode {
    public static double widthMulti = 0.5;

    @Override
    public void initialize() {
        super.initialize();

        camera.setPipeline(junctionWithAreaPipeline);

        try {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        }

        gb1(RIGHT_TRIGGER).whenActive(
                new InstantCommand(() -> {
                    if(junctionWithAreaPipeline.getRect() != null && !rrDrive.isBusy()) {
                        Rect rect = junctionWithAreaPipeline.getRect();
                        double junctionX = rect.x + (double) rect.width / 2;
                        rrDrive.turn(Math.toRadians((-(junctionX - 320) * pix_to_degree) / (widthMulti * rect.width)));
                    }
                })
        );
    }
    @Override
    public void run() {
        super.run();
        if(junctionWithAreaPipeline.getRect() != null)
            tad("change", -((junctionWithAreaPipeline.getRect().x + junctionWithAreaPipeline.getRect().width/2 - 320) * pix_to_degree));
        telemetry.update();
    }
}
