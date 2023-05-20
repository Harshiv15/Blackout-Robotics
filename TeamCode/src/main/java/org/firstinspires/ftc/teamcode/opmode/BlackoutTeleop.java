package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.TurnCommand;

@Config
@TeleOp
public class BlackoutTeleop extends BaseOpMode{
    private int ledState;

    @Override
    public void initialize() {
        super.initialize();

        gb1(LEFT_BUMPER).whileHeld(
                drive.slowMode(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX));

        gb1(X).toggleWhenPressed(
                drive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, imu::getAbsoluteHeading),
                drive.robotCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX)
        );

        gb1(DPAD_UP).whenPressed(
                new TurnCommand(rrDrive, 0)
        );
        gb1(DPAD_LEFT).whenPressed(
                new TurnCommand(rrDrive, 90)
        );
        gb1(DPAD_DOWN).whenPressed(
                new TurnCommand(rrDrive, 180)
        );
        gb1(DPAD_RIGHT).whenPressed(
                new TurnCommand(rrDrive, 270)
        );
        gb1(RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> ledState ++)
        );

        register(drive);
        drive.setDefaultCommand(drive.robotCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX));
    }

    @Override
    public void run() {
        super.run();
        tad("leDEEZ NUTS", ledState % 4);

        setLEDColors(ledState % 4);
        // Pronounced "headin-BRUH", obviously
        tad("headinburgh", imu.getHeading());
        telemetry.update();
    }
}
