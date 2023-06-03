package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDFElev extends OpMode {
    private PIDController controller;
    public static double    kP = 0.0,
                            kI = 0.0,
                            kD = 0.0,
                            kF = 0.0;

    public static int target = 0;

    public final double TICKS_IN_DEGREES = (28*12)/360.0;

    private DcMotorEx elevLeft, elevRight;

    @Override
    public void init() {
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevLeft = hardwareMap.get(DcMotorEx.class, "leftElevMotor");
        elevRight = hardwareMap.get(DcMotorEx.class, "rightElevMotor");

        elevLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(kP, kI, kD);
        int leftPos = elevLeft.getCurrentPosition();
        double pid = controller.calculate(target-leftPos);
        double ff = Math.cos(Math.toRadians(target/TICKS_IN_DEGREES)) * kF;

        double power = pid + ff;
        elevLeft.setPower(power);
        elevRight.setPower(power);

        telemetry.addData("leftPos", elevLeft.getCurrentPosition());
        telemetry.addData("rightPos", elevRight.getCurrentPosition());
        telemetry.addData("target", target);
    }
}
