package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;

public class AlignToPole extends CommandBase {
    private final MecanumDrive drive;
    private final JunctionWithArea pipeline;

    public AlignToPole(MecanumDrive drive, JunctionWithArea pipeline) {
        this.drive = drive;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return atTarget();
    }

    private boolean atTarget() {
        return true;
    }
}
