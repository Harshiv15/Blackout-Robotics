package org.firstinspires.ftc.teamcode.vision.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class JunctionAreaObserverTest extends LinearOpMode {
    private OpenCvCamera camera;
    private JunctionWithArea pipeline;
    private double pix_to_degree = 0.192;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new JunctionWithArea();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        telemetry.setMsTransmissionInterval(50);
        time.reset();

        while (!isStarted() && !isStopRequested()) {
            Rect rect;
            if ((rect = pipeline.getRect()) != null) {
                double x = rect.x + ((double) rect.width / 2);
                telemetry.addData("center", x);
                telemetry.addData("error", (x - 320));
                telemetry.addData("change", change(rect));

            } else {
                telemetry.addData("nothing seen", true);
            }
            telemetry.update();
        }
    }

    public double change (Rect rect){
        double junctionX = rect.x + (double) rect.width / 2;
        return ((junctionX - 320) * pix_to_degree);
    }
}
