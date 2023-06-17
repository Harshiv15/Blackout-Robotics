package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandBase;

public class NoRequirementInstantCommand extends CommandBase {

    private final Runnable m_toRun;

    /**
     * Creates a new InstantCommand that runs the given Runnable with the given requirements.
     *
     * @param toRun the Runnable to run
     */
    public NoRequirementInstantCommand(Runnable toRun) {
        m_toRun = toRun;
    }

    /**
     * Creates a new InstantCommand with a Runnable that does nothing.  Useful only as a no-arg
     * constructor to call implicitly from subclass constructors.
     */
    public NoRequirementInstantCommand() {
        m_toRun = () -> {
        };
    }

    @Override
    public void initialize() {
        m_toRun.run();
    }

    @Override
    public final boolean isFinished() {
        return true;
    }

}