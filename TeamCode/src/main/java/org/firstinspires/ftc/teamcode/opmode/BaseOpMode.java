package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class BaseOpMode extends CommandOpMode {
    protected MotorEx fL, fR, bL, bR;
    protected MecanumDriveSubsystem drive;
    /*protected ElevatorSubsystem elev;
    protected IntakeSubsystem intake;*/

    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;

    protected TriggerGamepadEx triggerGamepadEx1;
    protected TriggerGamepadEx triggerGamepadEx2;
    protected RevIMU imu;

    //protected SampleMecanumDrive rrDrive;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        initHardware();

        imu = new RevIMU(hardwareMap);
        imu.init();

        drive = new MecanumDriveSubsystem(fL, fR, bL, bR, imu);

        triggerGamepadEx1 = new TriggerGamepadEx(gamepad1, gamepadEx1);
        triggerGamepadEx2 = new TriggerGamepadEx(gamepad2, gamepadEx2);
    }
    protected void initHardware() {
        try {
            fL = hardwareMap.get(MotorEx.class, "frontLeft");
            fR = hardwareMap.get(MotorEx.class, "frontRight");
            bL = hardwareMap.get(MotorEx.class, "backLeft");
            bR = hardwareMap.get(MotorEx.class, "backRight");
        }
        catch(Exception e) {
            tad("ERROR", "Motor init failed");
        }
    }


    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        return new BigDecimal(Double.toString(value)).setScale(places, RoundingMode.HALF_UP).doubleValue();
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
