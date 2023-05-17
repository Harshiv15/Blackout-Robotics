package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp
public class BlackoutTeleop extends BaseOpMode{
    @Override
    public void initialize() {
        super.initialize();

        gb1(LEFT_BUMPER).whileHeld(
                drive.slowMode(gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));

        register(drive);
        drive.setDefaultCommand(drive.robotCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX));
    }
}
