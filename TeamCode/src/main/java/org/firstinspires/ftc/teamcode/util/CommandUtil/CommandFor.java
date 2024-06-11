package org.firstinspires.ftc.teamcode.util.CommandUtil;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;

import java.util.HashSet;
import java.util.Set;

/**
 * CommandFor is an abstract class that represents a generic command for a specific subsystem.
 * It implements the Command interface and provides a way to specify the subsystem requirements.
 *
 * @param <T> the type of subsystem this command is for, extending SubsystemBase
 * @see Command
 */
public abstract class CommandFor<T extends SubsystemBase> implements Command {
    protected final T subsystem;

    /**
     * Constructs a new CommandFor instance.
     *
     * @param subsystem the subsystem this command is for
     */
    public CommandFor(T subsystem) {
        this.subsystem = subsystem;
    }

    /**
     * Returns the set of subsystems required by this command.
     *
     * @return a set containing the required subsystems
     */
    @Override
    public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> set = new HashSet<>();
        set.add(subsystem);
        return set;
    }
}
