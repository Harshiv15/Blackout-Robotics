package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.powerplayutil.Height;

@Config
public class ElevatorSubsystem extends SubsystemBase {
    private final DcMotorEx leftElevMotor, rightElevMotor;

    private final double TICKS_IN_DEGREES = 8192.0 / 360;

    public static double kP = 0.0015; //0.0016
    public static double kI = 0.055; //0.06
    public static double kD = 0.0001; //0.00018
    public static double kF = 0.082; //0.06
    public static double maxVelocity = 6000;
    public static double maxAcceleration = 6000;

    private ProfiledPIDController leftPID = new ProfiledPIDController(kP, kI, kD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    private ProfiledPIDController rightPID = new ProfiledPIDController(kP, kI, kD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    public static double tolerance = 10;

    public static int currentHeight = 0;

    public static int manualLiftSpeed = 30;

    public static int threshold = 30;

    public ElevatorSubsystem(DcMotorEx leftElevMotor, DcMotorEx rightElevMotor) {
        this.leftElevMotor = leftElevMotor;
        this.rightElevMotor = rightElevMotor;
        leftPID.setTolerance(tolerance);
        leftPID.setGoal(0);
        rightPID.setGoal(0);
    }

    private void setHeight(Height height) {
        currentHeight = height.getHeight();
        leftPID.setGoal(height.getHeight());
        rightPID.setGoal(height.getHeight());
    }
}
