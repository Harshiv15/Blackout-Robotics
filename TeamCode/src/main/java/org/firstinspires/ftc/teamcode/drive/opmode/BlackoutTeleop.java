package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class BlackoutTeleop extends CommandOpMode {
    private DcMotorEx fL, fR, bL, bR;

    @Override
    public void initialize() {
        // Initialize hardware
        fL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fR = hardwareMap.get(DcMotorEx.class, "frontRight");
        bL = hardwareMap.get(DcMotorEx.class, "backLeft");
        bR = hardwareMap.get(DcMotorEx.class, "backRight");

        // Schedule all commands
        schedule();

        // Register unregistered subsystems
        register();
    }
}
