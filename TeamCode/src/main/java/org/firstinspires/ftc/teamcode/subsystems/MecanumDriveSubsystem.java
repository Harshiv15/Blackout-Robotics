package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.ProfiledAngleController;

import java.util.function.DoubleSupplier;

/**
 * A subsystem that uses the {@link SampleMecanumDrive} class.
 * This periodically calls {@link SampleMecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class MecanumDriveSubsystem extends SubsystemBase {
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

    public static double slowFactor = 14;

    private double target;

    public MecanumDriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, RevIMU imu) {
        this.imu = imu;
        fL.setInverted(true);
        bL.setInverted(true);
        drive = new MecanumDrive(false, fL, fR, bL, bR);
    }

    public Command fieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                DoubleSupplier turnSpeed, DoubleSupplier gyroAngle) {
        return new RunCommand(
                () -> drive.driveFieldCentric(strafeSpeed.getAsDouble()/7, forwardSpeed.getAsDouble()/7,
                        turnSpeed.getAsDouble(), gyroAngle.getAsDouble()/7),
                this
        );
    }

    public Command robotCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                DoubleSupplier turnSpeed) {
        return new RunCommand(
                () -> drive.driveRobotCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                        turnSpeed.getAsDouble(), true),
                this
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

}