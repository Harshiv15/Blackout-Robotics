package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.util.ProfiledPIDFController;

import java.util.function.DoubleSupplier;

@Config
public class ElevatorSubsystem extends SubsystemBase {
    private final MotorEx left, right;

    private final double TICKS_IN_DEGREES = (28*12)/360.0;

    public static double kP = 0.0075;
    public static double kI = 0; //0.06
    public static double kD = 0; //0.00018
    public static double kF = 0; //0.06
    public static double maxVelocity = 3000;
    public static double maxAcceleration = 3000;
    private ProfiledPIDFController m_PIDFController = new ProfiledPIDFController(kP, kI, kD, kF,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    public static double tolerance = 10;

    public static int currentHeight = 0;

    public static int manualLiftSpeed = 30;

    public static int threshold = 30;

    public ElevatorSubsystem(MotorEx left, MotorEx right) {
        this.left = left;
        this.right = right;
        left.setInverted(true);

        m_PIDFController.setTolerance(tolerance);
        // m_PIDFController.setGoal(0);
    }

    private void setHeight(Height height) {
        currentHeight = height.getHeight();
        m_PIDFController.setGoal(height.getHeight());
    }

    private void setHeight(int tick) {
        currentHeight = tick;
        m_PIDFController.setGoal(tick);
    }

    public int getLeftEncoderValue() {
        return left.getCurrentPosition();
    }

    public boolean atTarget() {
        return left.getCurrentPosition() < currentHeight + threshold &&
                left.getCurrentPosition() > currentHeight - threshold;
    }

    public int getCurrentGoal() {
        return currentHeight;
    }

    public Command goTo(Height height) {
        return new InstantCommand(() -> setHeight(height));
                //.andThen(new WaitUntilCommand(this::atTarget));
    }

    public Command goTo(int tick) {
        return new InstantCommand(() -> setHeight(tick));
                //.andThen(new WaitUntilCommand(this::atTarget));
    }

    public Command setPower(DoubleSupplier power) {
        return new RunCommand(() -> {
            if (Math.abs(power.getAsDouble()) > 0.01) {
                left.set(power.getAsDouble() * 2);
                right.set(power.getAsDouble() * 2);
                // m_PIDFController.setGoal(left.getCurrentPosition());
            } else {
                left.set(0);
                right.set(0);
            }
        }, this);
    }

    @Override
    public void periodic() {
        double output = m_PIDFController.calculate(left.getCurrentPosition());
        left.set(output);
//        Log.d("asd", "output left: "+ output_left);
        right.set(output);
    }

    public void setVelocityAccel(double maxVelocity, double maxAcceleration){
        m_PIDFController = new ProfiledPIDFController(kP, kI, kD, kF,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    }
}
