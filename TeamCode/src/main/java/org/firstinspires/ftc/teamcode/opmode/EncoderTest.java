package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class EncoderTest extends LinearOpMode {
    private DcMotor elevRight;

    @Override
    public void runOpMode() {
        elevRight = hardwareMap.get(DcMotorEx.class, "rightElevMotor");

        elevRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("", elevRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
