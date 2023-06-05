package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDFElev extends LinearOpMode {
    private PIDController controller;
    public static double    kP = 0.0,
                            kI = 0.0,
                            kD = 0.0,
                            kF = 0.0;

    public static int target = 0;

    public final double TICKS_IN_DEGREES = (28*12)/360.0;

    private DcMotor elevLeft, elevRight;

    @Override
    public void runOpMode() {
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // elevLeft = hardwareMap.get(DcMotorEx.class, "leftElevMotor");
        elevRight = hardwareMap.get(DcMotor.class, "rightElevMotor");

        // elevLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        elevRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()) {
            controller.setPID(kP, kI, kD);
            int rightPos = elevRight.getCurrentPosition();
            double pid = Math.abs(target - rightPos) > 10 ? controller.calculate(rightPos, target * TICKS_IN_DEGREES) : 0;
            double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES)) * kF;

            double power = pid + ff;
            // elevLeft.setPower(power);
            elevRight.setPower(power);

            // telemetry.addData("leftPos", elevLeft.getCurrentPosition());
            telemetry.addData("pid", pid);
            telemetry.addData("rightPos", elevRight.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.update();
        }
    }

}
