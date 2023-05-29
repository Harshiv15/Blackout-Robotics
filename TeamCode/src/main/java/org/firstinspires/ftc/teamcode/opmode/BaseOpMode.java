package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionObserverPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class BaseOpMode extends CommandOpMode {
    protected MotorEx fL, fR, bL, bR, elevLeft, elevRight;
    protected MecanumDriveSubsystem drive;
    protected SampleMecanumDrive rrDrive;
    protected JunctionObserverPipeline pipeline;
    protected OpenCvCamera camera;
    protected ElevatorSubsystem elev;
    /*protected ElevatorSubsystem elev;
    protected IntakeSubsystem intake;*/

    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;

    protected TriggerGamepadEx triggerGamepadEx1;
    protected TriggerGamepadEx triggerGamepadEx2;
    protected RevIMU imu;

    protected DigitalChannel red0, green0, red1, green1, red2, green2, red3, green3;
    protected DigitalChannel red4, green4, red5, green5, red6, green6, red7, green7;

    protected DigitalChannel[][] indicators = new DigitalChannel[8][2];

    //protected SampleMecanumDrive rrDrive;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        initHardware();
        setupHardware();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        pipeline = new JunctionObserverPipeline();
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        camera.setPipeline(pipeline);

        imu = new RevIMU(hardwareMap);
        imu.init();

        drive = new MecanumDriveSubsystem(fL, fR, bL, bR, imu);
        rrDrive = new SampleMecanumDrive(hardwareMap);
        rrDrive.setPoseEstimate(new Pose2d(36, 60, Math.toRadians(-90)));

        triggerGamepadEx1 = new TriggerGamepadEx(gamepad1, gamepadEx1);
        triggerGamepadEx2 = new TriggerGamepadEx(gamepad2, gamepadEx2);

        elev = new ElevatorSubsystem(elevLeft, elevRight);
    }
    protected void initHardware() {
        try {
            fL = new MotorEx(hardwareMap, "frontLeft");
            fR = new MotorEx(hardwareMap, "frontRight");
            bL = new MotorEx(hardwareMap, "backLeft");
            bR = new MotorEx(hardwareMap, "backRight");
            elevLeft = new MotorEx(hardwareMap, "leftElevMotor");
            elevRight = new MotorEx(hardwareMap, "rightElevMotor");

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

            /*for(int row = 0; row < indicators.length; row++) {
                for(int color = 0; color < 2; color++) {
                    indicators[row][color] = hardwareMap.get(DigitalChannel.class, color == 0 ? "green" : "red" + row);
                }
            }*/
        }
        catch(Exception e) {
            tad("ERROR", "Motor init failed");
        }
    }

    protected void setupHardware() {
        fL.setInverted(true);
        bL.setInverted(true);
        elevLeft.setInverted(true);

        elevLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        elevRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        red0.setMode(DigitalChannel.Mode.OUTPUT);
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
        green7.setMode(DigitalChannel.Mode.OUTPUT);

        elevLeft.resetEncoder();
        elevRight.resetEncoder();

        /*for(DigitalChannel[] row : indicators) {
            for(DigitalChannel led : row) {
                led.setMode(DigitalChannel.Mode.OUTPUT);
            }
        }*/
    }


    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        return new BigDecimal(Double.toString(value)).setScale(places, RoundingMode.HALF_UP).doubleValue();
    }

    public void setLEDColors(int colorId) {
        boolean red, green;
        if (colorId == 1) {
            red = false;
            green = true;
        } else if (colorId == 2) {
            red = true;
            green = false;
        } else if (colorId == 3) {
            red = false;
            green = false;
        } else {
            red = true;
            green = true;
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

        /*for(DigitalChannel[] row : indicators) {
            for(int i = 0; i < row.length; i++) {
                row[i].setState(i % 0 == 0 ? red : green);
            }
        }*/
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
}
