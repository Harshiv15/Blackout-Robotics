package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;

public class BaseOpMode extends CommandOpMode {
    protected MotorEx fL, fR, bL, bR, elevLeft, elevRight;
    // * for resetting encoders bc MotorEx sux
    protected DcMotorEx elevLeftDC, elevRightDC;
    protected SimpleServo armLeft, armRight;
    protected Servo clawLeft, clawRight;
    protected MecanumDriveSubsystem drive;
    protected SampleMecanumDrive rrDrive;
    protected JunctionWithArea junctionWithAreaPipeline;
    protected AprilTagDetectionPipeline aprilTagDetectionPipeline;
    protected OpenCvCamera camera;
    protected ElevatorSubsystem elev;
    protected ClawSubsystem claw;
    protected ArmSubsystem arm;

    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;

    protected TriggerGamepadEx triggerGamepadEx1;
    protected TriggerGamepadEx triggerGamepadEx2;
    protected RevIMU imu;

    protected DigitalChannel red0, green0, red1, green1, red2, green2, red3, green3;
    protected DigitalChannel red4, green4, red5, green5, red6, green6, red7, green7;

    protected DigitalChannel[][] indicators = new DigitalChannel[8][2];

    protected double fx = 578.272;
    protected double fy = 578.272;
    protected double cx = 402.145;
    protected double cy = 221.506;

    // UNITS ARE METERS
    protected double tagsize = 0.166;
    protected AprilTagDetection tagOfInterest = null;
    public static int LEFT = 1, MIDDLE = 2, RIGHT = 3;
    static final double FEET_PER_METER = 3.28084;


    @Override
    public void initialize() {
        tad("Mode", "Starting initialization");
        telemetry.update();
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        initHardware();
        setupHardware();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        junctionWithAreaPipeline = new JunctionWithArea();
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        //camera.setPipeline(junctionWithAreaPipeline);
        camera.setPipeline(aprilTagDetectionPipeline);

        imu = new RevIMU(hardwareMap);
        imu.init();

        drive = new MecanumDriveSubsystem(fL, fR, bL, bR, imu);
        rrDrive = new SampleMecanumDrive(hardwareMap);
        rrDrive.setPoseEstimate(new Pose2d(-36, -63, Math.toRadians(-90)));

        triggerGamepadEx1 = new TriggerGamepadEx(gamepad1, gamepadEx1);
        triggerGamepadEx2 = new TriggerGamepadEx(gamepad2, gamepadEx2);

        elev = new ElevatorSubsystem(elevLeft, elevRight);
        arm = new ArmSubsystem(armLeft, armRight);
        claw = new ClawSubsystem(clawLeft, clawRight);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tad("Mode", "Done initializing");

        telemetry.update();
    }

    protected void detectSleeve() {

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if (tagFound) {
                tal("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            } else {
                tal("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    tal("(The tag has never been seen)");
                } else {
                    tal("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        } else {
            tal("Don't see tag of interest :(");

            if (tagOfInterest == null) {
                tal("(The tag has never been seen)");
            } else {
                tal("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        sleep(20);
        telemetry.update();
    }

    protected void tagToTelemetry(AprilTagDetection detection)
    {
        tal(String.format("\nDetected tag ID=%d", detection.id));
        tal(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        tal(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        tal(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }

    protected void initHardware() {
        try {
            fL = new MotorEx(hardwareMap, "frontLeft");
            fR = new MotorEx(hardwareMap, "frontRight");
            bL = new MotorEx(hardwareMap, "backLeft");
            bR = new MotorEx(hardwareMap, "backRight");
            elevLeft = new MotorEx(hardwareMap, "leftElevMotor");
            elevRight = new MotorEx(hardwareMap, "rightElevMotor");
            elevLeftDC = hardwareMap.get(DcMotorEx.class, "leftElevMotor");
            elevRightDC = hardwareMap.get(DcMotorEx.class, "rightElevMotor");
            armLeft = new SimpleServo(hardwareMap, "armLeft", 0, 355);
            armRight = new SimpleServo(hardwareMap, "armRight", 0, 355);
            clawLeft = hardwareMap.get(Servo.class, "clawLeft");
            clawRight = hardwareMap.get(Servo.class, "clawRight");

            red0 = hardwareMap.get(DigitalChannel.class, "red0");
            green0 = hardwareMap.get(DigitalChannel.class, "green0");
            red1 = hardwareMap.get(DigitalChannel.class, "red1");
            green1 = hardwareMap.get(DigitalChannel.class, "green1");
            red2 = hardwareMap.get(DigitalChannel.class, "red2");
            green2 = hardwareMap.get(DigitalChannel.class, "green2");
            red3 = hardwareMap.get(DigitalChannel.class, "red3");
            green3 = hardwareMap.get(DigitalChannel.class, "green3");
            red4 = hardwareMap.get(DigitalChannel.class, "red4");
            green4 = hardwareMap.get(DigitalChannel.class, "green4");
            red5 = hardwareMap.get(DigitalChannel.class, "red5");
            green5 = hardwareMap.get(DigitalChannel.class, "green5");
            red6 = hardwareMap.get(DigitalChannel.class, "red6");
            green6 = hardwareMap.get(DigitalChannel.class, "green6");
            red7 = hardwareMap.get(DigitalChannel.class, "red7");
            green7 = hardwareMap.get(DigitalChannel.class, "green7");
        }
        catch(Exception e) {
            tad("ERROR", "Motor init failed");
        }
    }

    protected void setupHardware() {
        fL.setInverted(true);
        bL.setInverted(true);

        /*red0.setMode(DigitalChannel.Mode.OUTPUT);
        green0.setMode(DigitalChannel.Mode.OUTPUT);
        red1.setMode(DigitalChannel.Mode.OUTPUT);
        green1.setMode(DigitalChannel.Mode.OUTPUT);
        red2.setMode(DigitalChannel.Mode.OUTPUT);
        green2.setMode(DigitalChannel.Mode.OUTPUT);
        red3.setMode(DigitalChannel.Mode.OUTPUT);
        green3.setMode(DigitalChannel.Mode.OUTPUT);
        red4.setMode(DigitalChannel.Mode.OUTPUT);
        green4.setMode(DigitalChannel.Mode.OUTPUT);
        red5.setMode(DigitalChannel.Mode.OUTPUT);
        green5.setMode(DigitalChannel.Mode.OUTPUT);
        red6.setMode(DigitalChannel.Mode.OUTPUT);
        green6.setMode(DigitalChannel.Mode.OUTPUT);
        red7.setMode(DigitalChannel.Mode.OUTPUT);
        green7.setMode(DigitalChannel.Mode.OUTPUT);*/

        elevLeftDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevRightDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevLeftDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevRightDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevLeft.resetEncoder();
        elevRight.resetEncoder();

        // * using encoder???
        elevLeft.setRunMode(Motor.RunMode.RawPower);
        elevRight.setRunMode(Motor.RunMode.RawPower);
        elevLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        elevRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        return new BigDecimal(Double.toString(value)).setScale(places, RoundingMode.HALF_UP).doubleValue();
    }

    public void setLEDColors(int colorId) {
        boolean red, green;
        switch (colorId) {
            case 1:
                red = false;
                green = true;
                break;
            case 2:
                red = true;
                green = false;
                break;
            case 3:
                red = false;
                green = false;
                break;
            default:
                red = true;
                green = true;
                break;
        }

        red0.setState(red);
        red1.setState(red);
        red2.setState(red);
        red3.setState(red);
        red4.setState(red);
        red5.setState(red);
        red6.setState(red);
        red7.setState(red);
        green0.setState(green);
        green1.setState(green);
        green2.setState(green);
        green3.setState(green);
        green4.setState(green);
        green5.setState(green);
        green6.setState(green);
        green7.setState(green);
    }

    private static double round(double value) {
        return round(value, 4);
    }

    public GamepadButton gb1(GamepadKeys.Button button) {
        return  gamepadEx1.getGamepadButton(button);
    }


    public GamepadButton gb2(GamepadKeys.Button button) {
        return  gamepadEx2.getGamepadButton(button);
    }


    public GamepadTrigger gb1(GamepadKeys.Trigger trigger) {
        return  triggerGamepadEx1.getGamepadTrigger(trigger);
    }


    public GamepadTrigger gb2(GamepadKeys.Trigger trigger) {
        return  triggerGamepadEx1.getGamepadTrigger(trigger);
    }


    // telemetry add data = tad
    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    protected void tal(String caption) {
        telemetry.addLine(caption);
    }
}
