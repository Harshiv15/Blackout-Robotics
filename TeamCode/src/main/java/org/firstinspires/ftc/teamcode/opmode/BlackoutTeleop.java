package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.util.TurnCommand;
import org.firstinspires.ftc.teamcode.vision.util.CVMaster;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp
public class BlackoutTeleop extends BaseOpMode{
    private int ledState;

    @Override
    public void initialize() {
        super.initialize();

        CVMaster cv = new CVMaster(this);
        cv.observeStick();

        /*camera.setPipeline(pipeline);

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
        }*/


        cv.observeStick();

        elev.goTo(Height.HIGH);

        gb1(LEFT_BUMPER).whileHeld(
                drive.slowMode(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX));

        gb1(START).toggleWhenPressed(
                drive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, imu::getAbsoluteHeading),
                drive.robotCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX)
        );

        gb2(Y).whenActive(elev.goTo(Height.HIGH));
        gb2(B).whenActive(elev.goTo(Height.MEDIUM));
        gb2(X).whenActive(elev.goTo(Height.LOW));
        gb2(A).whenActive(elev.goTo(Height.GROUND));

        register(drive, elev);
        drive.setDefaultCommand(drive.robotCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX));
        elev.setDefaultCommand(elev.setPower(gamepadEx2::getLeftY));
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
