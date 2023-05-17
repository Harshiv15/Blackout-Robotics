package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;

public class BaseOpMode extends CommandOpMode {
    protected DcMotorEx frontLeft, frontRight, backLeft, backRight;
    protected MecanumDriveSubsystem drive;
    /*protected ElevatorSubsystem elev;
    protected IntakeSubsystem intake;*/

    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;

    protected TriggerGamepadEx triggerGamepadEx1;
    protected TriggerGamepadEx triggerGamepadEx2;

    //protected SampleMecanumDrive rrDrive;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        triggerGamepadEx1 = new TriggerGamepadEx(gamepad1, gamepadEx1);
        triggerGamepadEx2 = new TriggerGamepadEx(gamepad2, gamepadEx2);
    }
    protected void initHardware() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
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
