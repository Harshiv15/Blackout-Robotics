package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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
    public static double maxVelocity = 6000;
    public static double maxAcceleration = 6000;
    private ProfiledPIDFController leftPIDF = new ProfiledPIDFController(kP, kI, kD, kF,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    private ProfiledPIDFController rightPIDF = new ProfiledPIDFController(kP, kI, kD, kF,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    public static double tolerance = 10;

    public static int currentHeight = 0;

    public static int threshold = 30;
    private boolean setpoints = true;
    private double leftMotorPower, rightMotorPower;

    public ElevatorSubsystem(MotorEx left, MotorEx right) {
        this.left = left;
        this.right = right;
        left.setInverted(true);

        leftPIDF.setTolerance(tolerance);
        rightPIDF.setTolerance(tolerance);
        leftPIDF.setGoal(0);
        rightPIDF.setGoal(0);
    }

    private void setHeight(Height height) {
        currentHeight = height.getHeight();
        leftPIDF.setGoal(height.getHeight());
        rightPIDF.setGoal(height.getHeight());
    }

    private void setHeight(int tick) {
        currentHeight = tick;
        leftPIDF.setGoal(tick);
        rightPIDF.setGoal(tick);
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
        return new InstantCommand(()-> setHeight(height), this)
                .andThen(new WaitUntilCommand(this::atTarget));
    }

    public Command goTo(int tick) {
        return new InstantCommand(() -> setHeight(tick), this)
                .andThen(new WaitUntilCommand(this::atTarget));
    }

    public Command setPower(DoubleSupplier power) {
        return new RunCommand(() -> {
            if (Math.abs(power.getAsDouble()) > 0.01) {
                leftMotorPower = power.getAsDouble()*2;
                rightMotorPower = power.getAsDouble()*2;
                //leftPIDF.setGoal(left.getCurrentPosition());
                //rightPIDF.setGoal(left.getCurrentPosition());
            }
        }, this);
    }

    public Command setPower(double power) {
        return new RunCommand(() -> {
           if(Math.abs(power) > 0.01) {
               leftMotorPower = power*2;
               rightMotorPower = power*2;
           }
        }, this);
    }

    public void startSetpoints() {
        setpoints = true;
    }

    public void stopSetpoints() {
        setpoints = false;
    }

    private void stop() {
        left.set(0);
        right.set(0);
    }

    private void resetEncoders() {
        left.resetEncoder();
        right.resetEncoder();
    }


    @Override
    public void periodic() {
        /*if(setpoints) {
            double outputL = leftPIDF.calculate(left.getCurrentPosition());
            double outputR = rightPIDF.calculate(right.getCurrentPosition());
            left.set(outputL);
            right.set(outputR);
        }
        else {*/
            double output_left = leftPIDF.calculate(left.getCurrentPosition());
            double output_right = rightPIDF.calculate(right.getCurrentPosition());
            left.set(output_left);
            right.set(output_right);
            super.periodic();
        //}

    }


    public void setVelocityAccel(double maxVelocity, double maxAcceleration){
        leftPIDF = new ProfiledPIDFController(kP, kI, kD, kF,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    }
}
