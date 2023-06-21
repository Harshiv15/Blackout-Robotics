package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.HEADING_PID;
import static org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem.calculateDSq;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.opencv.core.Rect;

public class AlignToPole extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final RevIMU imu;
    private final JunctionWithArea pipeline;
    private final PIDController controller;
    public static double kF = 0;
    protected final ElapsedTime time = new ElapsedTime();
    Rect rect;
    private double target;
    private double change;
    double error;
    private double pix_to_degree = 0.192;

    public AlignToPole(MecanumDriveSubsystem drive, JunctionWithArea pipeline, RevIMU imu) {
        this.drive = drive;
        this.imu = imu;
        this.pipeline = pipeline;
        controller = new PIDController(HEADING_PID.kP, HEADING_PID.kI, HEADING_PID.kD);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if((rect = pipeline.getRect()) != null) {
            updateTarget(rect);
            error = target - getYaw();
            change = controller.calculate(calculateDSq(error, time));
            time.reset();
            drive.driveWithMotorPowers(change,-change,-change,change);
        }
    }

    @Override
    public boolean isFinished() {
        return pipeline.getRect() == null || Math.abs(target - (rect = pipeline.getRect()).x) < 3;
    }

    public void updateTarget(Rect rect) {
        double junctionX = rect.x + (double) rect.width / 2;
        target = (getYaw() + ((junctionX - 160) * pix_to_degree));
    }

    private double getYaw() {
        return imu.getHeading();
    }
}
