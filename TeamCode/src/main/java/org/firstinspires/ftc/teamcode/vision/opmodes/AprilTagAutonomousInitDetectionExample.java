/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;

    // UNITS ARE METERS
    public static double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    private IMU imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo clawLeft;
    private Servo clawRight;
    private Servo armLeft;
    private Servo armRight;
    private DcMotor leftElevMotor;
    private DcMotor rightElevMotor;

    double power;
    double elevSpeed;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        imu = hardwareMap.get(IMU.class, "imu");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        leftElevMotor = hardwareMap.get(DcMotor.class, "leftElevMotor");
        rightElevMotor = hardwareMap.get(DcMotor.class, "rightElevMotor");

        // Put initialization blocks here.
        power = 0.5;
        elevSpeed = 0.7;
        leftElevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        armLeft.setDirection(Servo.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        imu.resetYaw();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */

        if(tagOfInterest == null){
            //default trajectory here if preferred
        }else if(tagOfInterest.id == LEFT){
            //left trajectory
        }else if(tagOfInterest.id == MIDDLE){
            //middle trajectory
        }else{
            //right trajectory
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }
    /**
     * Describe this function...
     */
    private void cycle() {
        elevToEncTicks(-(1150 + 40));
        armToPos(0.9);
        turnCWWithRevs(-1.65);
        sleep(1500);
        forwardWithRevs(1.8);
        sleep(1500);
        grab();
        sleep(250);
        armToPos(0.5);
        forwardWithRevs(-1);
        sleep(500);
        strafeRightToRevs(-0.4);
        sleep(100);
        forwardWithRevs(-2.5);
        sleep(1000);
        turnCWWithRevs(-1.65);
        elevToEncTicks(1150 + 40);
        sleep(1500);
        armToPos(0.5);
        armToPos(0.4);
        forwardWithRevs(-0.8);
        sleep(500);
        elevToEncTicks(-600);
        sleep(500);
        deposit();
        sleep(500);
        forwardWithRevs(0.4);
        armToPos(0.9);
        elevToEncTicks(600);
        sleep(750);
        elevSpeed = 0.5;
    }

    /**
     * Describe this function...
     */
    private void deposit() {
        release();
        sleep(100);
        grab();
        sleep(100);
    }

    /**
     * Scores the preload on the high pole closest to the cone stack
     */
    private void preload() {
        grab();
        sleep(1000);
        forwardWithRevs(4.1);
        elevToEncTicks(1440);
        armToPos(0.6);
        sleep(2500);
        strafeRightToRevs(1.2);
        sleep(1000);
        forwardWithRevs(0.2);
        sleep(500);
        elevSpeed = 0.8;
        elevToEncTicks(-600);
        sleep(500);
        release();
        forwardWithRevs(-0.2);
        elevToEncTicks(600);
        armToPos(0.6);
        sleep(750);
        elevSpeed = 0.5;
        forwardWithRevs(-0.4);
        sleep(250);
        strafeRightToRevs(-1.5);
        sleep(1000);
    }

    /**
     * Describe this function...
     */
    private void grab() {
        clawLeft.setPosition(0.325);
        clawRight.setPosition(0.275);
    }

    /**
     * Describe this function...
     */
    private void armToPos(double pos) {
        armLeft.setPosition(pos);
        armRight.setPosition(pos);
    }

    /**
     * Describe this function...
     */
    private void strafeRightToRevs(double revsStrafe) {
        driveToEncTicks(revsStrafe * 560, -revsStrafe * 560, -revsStrafe * 560, revsStrafe * 560);
    }

    /**
     * Describe this function...
     */
    private void elevToEncTicks(int ticksElev) {
        leftElevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevMotor.setPower(elevSpeed);
        rightElevMotor.setPower(elevSpeed);
        leftElevMotor.setTargetPosition(ticksElev);
        rightElevMotor.setTargetPosition(ticksElev);
        leftElevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void release() {
        clawLeft.setPosition(0.45);
        clawRight.setPosition(0);
    }

    /**
     * Describe this function...
     */
    private void driveToRevs(double revsFL, double revsFR, double revsBL, double revsBR) {
        driveToEncTicks(revsFL * 560, revsFR * 560, revsBL * 560, revsBR * 560);
    }

    /**
     * Describe this function...
     */
    private void driveToEncTicks(double ticksFL, double ticksFR, double ticksBL, double ticksBR) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setTargetPosition((int) ticksFL);
        frontRight.setTargetPosition((int) ticksFR);
        backLeft.setTargetPosition((int) ticksBL);
        backRight.setTargetPosition((int) ticksBR);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void forwardWithRevs(double revsForward) {
        driveToRevs(revsForward, revsForward, revsForward, revsForward);
    }

    /**
     * Describe this function...
     */
    private void turnCWWithRevs(double revsTurn) {
        driveToRevs(revsTurn, -revsTurn, revsTurn, -revsTurn);
    }
}