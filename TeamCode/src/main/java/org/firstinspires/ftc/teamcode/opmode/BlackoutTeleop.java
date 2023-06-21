package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp
public class BlackoutTeleop extends BaseOpMode{
    private int ledState;
    private double deadzone = 0.01;
    private boolean driveDeadzoned, elevDeadzoned, armDeadzoned;
    Rect rect;

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

        gb1(LEFT_BUMPER).whileHeld(
                drive.slowMode(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX)
        );

        gb1(START).toggleWhenPressed(
                drive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, imu::getHeading),
                drive.robotCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX)
        );

        gb1(RIGHT_TRIGGER).whileActiveContinuous(
                drive.alignToPole(junctionWithAreaPipeline)
        );

        gb1(START).and(gb1(RIGHT_TRIGGER)).toggleWhenActive(
                new InstantCommand(), true
        );

        gb2(LEFT_BUMPER).whenActive(
                claw.grab().andThen(arm.back())
        );

        gb2(RIGHT_BUMPER).whenActive(
                claw.grab().andThen(arm.front())
        );

        gb2(LEFT_TRIGGER).or(gb2(RIGHT_TRIGGER)).whenActive(
                new InstantCommand(elev::stopSetpoints)
        );

        gb2(Y).or(gb2(B)).or(gb2(X)).or(gb2(A)).whenActive(
                new InstantCommand(elev::startSetpoints)
        );

        gb2(LEFT_TRIGGER).or(gb2(RIGHT_TRIGGER)).whileActiveContinuous(
                elev.setPower(gamepadEx2.getTrigger(RIGHT_TRIGGER)-gamepadEx2.getTrigger(LEFT_TRIGGER))
        );

        gb2(DPAD_RIGHT).toggleWhenPressed(
                claw.grab().andThen(new DelayedCommand(arm.idle(), 500)),
                arm.back().andThen(new DelayedCommand(claw.release(),100))
                        .andThen(new DelayedCommand(claw.grab(), 50))
                        .andThen(new DelayedCommand(arm.front(), 100))
                        .andThen(claw.release())
                        .andThen(elev.goTo(Height.NONE))
        );

        gb2(Y).whenActive(elev.goTo(Height.HIGH));
        gb2(B).whenActive(elev.goTo(Height.MEDIUM));
        gb2(X).whenActive(elev.goTo(Height.LOW));
        gb2(A).whenActive(elev.goTo(Height.GROUND));

        register(drive, elev, claw, arm);
        drive.setDefaultCommand(drive.robotCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX));
        elev.setDefaultCommand(elev.setPower(gamepadEx2.getTrigger(RIGHT_TRIGGER)-gamepadEx2.getTrigger(LEFT_TRIGGER)));
    }

    @Override
    public void run() {
        super.run();
        tad("leDEEZ NUTS", ledState % 4);
        rrDrive.update();

        // setLEDColors(ledState % 4);
        // Pronounced "headin-BRUH", obviously
        tad("headinburgh", imu.getHeading());
        tad("target", drive.getTarget());
        if(junctionWithAreaPipeline.getRect() != null)
            tad("closest X",
                    "" + Math.abs((rect = junctionWithAreaPipeline.getRect()).x + (rect = junctionWithAreaPipeline.getRect()).width/2 - 320)
        );

        telemetry.update();
    }
}
