package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.HEADING_PID;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AngleController;
import org.firstinspires.ftc.teamcode.util.NoRequirementInstantCommand;
import org.firstinspires.ftc.teamcode.util.ProfiledAngleController;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.opencv.core.Rect;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * A subsystem that uses the {@link SampleMecanumDrive} class.
 * This periodically calls {@link SampleMecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
@Config
public class MecanumDriveSubsystem extends SubsystemBase {
    public static double kF = 5;
    private final MecanumDrive drive;
    private final RevIMU imu;
    public static double kP = 0.06;
    public static double kI = 0;
    public static double kD = 0.0035;
    public static double maxVelocity = 20;
    public static double maxAcceleration = 20;
    private final ProfiledAngleController controller = new ProfiledAngleController(kP, kI, kD, 0,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    private double output;

    public static int joystickTransformFactor = 30;

    private double target = 0;

    public static double slowFactor = 2;
    public static double pix_to_degree = 0.192;
    Rect rect;
    private ElapsedTime time = new ElapsedTime();
    public double change;
    public static double multiplier;

    public MecanumDriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, RevIMU imu) {
        this.imu = imu;
        fL.setInverted(true);
        bL.setInverted(true);
        drive = new MecanumDrive(false, fL, fR, bL, bR);
    }

    public Command fieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                DoubleSupplier turnSpeed, DoubleSupplier gyroAngle) {
        return new RunCommand(
                () -> drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                        turnSpeed.getAsDouble(), gyroAngle.getAsDouble()),
                this
        );
    }

    public Command robotCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                DoubleSupplier turnSpeed) {
        return new RunCommand(
                () -> drive.driveRobotCentric(joystickTransform(strafeSpeed.getAsDouble()), joystickTransform(forwardSpeed.getAsDouble()),
                        joystickTransform(turnSpeed.getAsDouble()), false),
                this
        );
    }

    public Command alignToPole(@NonNull JunctionWithArea pipeline) {
        return new ConditionalCommand (
                new NoRequirementInstantCommand(()->{
                    // idle
                }),
                new RunCommand(()->{
                    if((rect = pipeline.getRect()) != null) {
                        updateTarget(rect);
                        double error = target - getYaw();
                        change = (controller.calculate(error))*multiplier;
                        drive.driveWithMotorPowers(change,-change,change,-change);
                    }
                }, this),
                /*() -> pipeline.getRect() == null ||
                        Math.abs((rect = pipeline.getRect()).x + (rect = pipeline.getRect()).width/2 - 320) < 640/(rect = pipeline.getRect()).width*/
                () -> {
                    if(pipeline.getRect() != null) {
                        Rect temp = pipeline.getRect();
                        return Math.abs(temp.x + temp.width/2 - 320) < 160/temp.width;
                    }
                    return true;
                }
        );
    }

    public Command slowMode(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                            DoubleSupplier turnSpeed) {
        return new RunCommand(
                () -> drive.driveRobotCentric(strafeSpeed.getAsDouble() / slowFactor,
                        forwardSpeed.getAsDouble() / slowFactor,
                        turnSpeed.getAsDouble() / slowFactor),
                this
        );
    }

    public void driveWithMotorPowers(double fL, double fR, double bL, double bR) {
        drive.driveWithMotorPowers(fL, fR, bL, bR);
    }

    public double getOutput() {
        return output;
    }

    public double getTarget() {
        return target;
    }

    // desmos: https://www.desmos.com/calculator/j2e6yaorld
    public double joystickTransform(double input) {
        return (1.0 / (joystickTransformFactor - 1))
                * Math.signum(input)
                * (Math.pow(joystickTransformFactor, Math.abs(input)) - 1);
    }

    public void updateTarget(Rect rect) {
        double junctionX = rect.x + (double) rect.width / 2;
        target = (getYaw() + ((junctionX - 320) * pix_to_degree));
    }

    private double getYaw() {
        return imu.getHeading();
    }

    public static double calculateDSq(double error, ElapsedTime time){
        double errorVal_v;
        if (time.milliseconds() > 1E-6) {
            errorVal_v = (error) / time.milliseconds();
        } else {
            errorVal_v = 0;
        }

        double targetVel = error * Math.abs(error) * HEADING_PID.kP + kF * error / Math.abs(error) + HEADING_PID.kD * errorVal_v;
        //if(targetVel > MAX_VELOCITY) targetVel = MAX_VELOCITY;
        return targetVel*time.seconds();
    }

    public double getChange() {
        return change;
    }
}