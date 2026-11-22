package frc.robot.commands.compat;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

/**
 * A Commands v2-style command (initialize / execute / isFinished / end) that runs on the Commands v3
 * scheduler.
 *
 * <p>TEMPORARY migration aid: existing v2 command classes switch to this base class by changing
 * only their import. Each one is later rewritten as a native v3 command, and this package is
 * deleted once nothing uses it.
 *
 * <p>Lifecycle, matching v2's {@code CommandScheduler}:
 * <ol>
 * <li>{@link #initialize()} once when the command starts.
 * <li>Each loop: {@link #execute()}, then {@link #isFinished()}. {@code execute()} always runs at
 * least once before the first {@code isFinished()} check.
 * <li>{@link #end(boolean) end(false)} when {@code isFinished()} returns true, or
 * {@link #end(boolean) end(true)} if the command is canceled or interrupted (by a conflicting
 * command, a timeout, a race, or its parent group ending). {@code end} is only called if
 * {@code initialize} was.
 * </ol>
 */
public abstract class Command implements org.wpilib.command3.Command {
	/**
	 * What happens when another command that needs the same subsystem is scheduled while this one
	 * runs. Same names as v2.
	 */
	public enum InterruptionBehavior {
		/** The new command interrupts this one (v2's default). */
		CANCEL_SELF,
		/** The new command is refused and this one keeps running. */
		CANCEL_INCOMING
	}

	/**
	 * v3 has no interruption behaviors, only priorities: a running command is interrupted by a new
	 * conflicting command of the same or higher priority, but not by a lower one. CANCEL_INCOMING
	 * commands get this priority, one above the default, so ordinary commands can't interrupt them.
	 */
	static final int CANCEL_INCOMING_PRIORITY = DEFAULT_PRIORITY + 1;

	private final Set<Mechanism> m_requirements = new LinkedHashSet<>();
	private int m_priority = DEFAULT_PRIORITY;
	private String m_name;
	// True between initialize() and end(), so a cancel that arrives before the command ever started
	// does not call end(true) (v2 never calls end without a matching initialize).
	private boolean m_started;

	protected Command() {
		// Same default as v2: the simple class name
		String name = getClass().getSimpleName();
		m_name = name.isEmpty() ? "Command" : name;
	}

	/** Called once when the command starts. */
	public void initialize() {
	}

	/** Called every loop while the command is running. */
	public void execute() {
	}

	/**
	 * Whether the command has finished. Checked every loop after {@link #execute()}.
	 *
	 * @return true to end the command
	 */
	public boolean isFinished() {
		return false;
	}

	/**
	 * Called once when the command ends.
	 *
	 * @param interrupted false if {@link #isFinished()} returned true; true if the command was
	 *        canceled or interrupted
	 */
	public void end(boolean interrupted) {
	}

	/**
	 * Adds mechanisms (subsystems) this command uses. The scheduler interrupts any other command that
	 * uses one of them when this command starts, as in v2.
	 *
	 * @param requirements the mechanisms to require
	 */
	public final void addRequirements(Mechanism... requirements) {
		for (Mechanism requirement : requirements) {
			if (requirement != null) {
				m_requirements.add(requirement);
			}
		}
	}

	public String getName() {
		return m_name;
	}

	public void setName(String name) {
		m_name = name;
	}

	/**
	 * Creates a command that runs this one for at most {@code seconds}, after which this command is
	 * interrupted ({@code end(true)}). Same as v2's {@code withTimeout(double)}.
	 *
	 * @param seconds the timeout in seconds
	 * @return the decorated command
	 */
	public Command withTimeout(double seconds) {
		// Not v3's withTimeout(Time): it is built on v3's race, which has the run-twice bug
		// described in Commands.race().
		Command timed = Commands.race(this, Commands.waitSeconds(seconds));
		timed.setName(getName() + " [" + seconds + " s timeout]");
		return timed;
	}

	/**
	 * Creates a command that behaves exactly like this one but has a different name. Same as v2's
	 * {@code withName(String)}.
	 *
	 * @param name the new name
	 * @return the decorated command
	 */
	public Command withName(String name) {
		Command wrapped = wrap(this);
		wrapped.setName(name);
		return wrapped;
	}

	/**
	 * Creates a command that behaves exactly like this one but with a different interruption
	 * behavior. Same as v2's {@code withInterruptBehavior}, with one difference: two
	 * {@code CANCEL_INCOMING} commands that need the same subsystem still interrupt each other,
	 * because v3 lets a command interrupt one of equal priority.
	 *
	 * @param behavior what happens when a conflicting command is scheduled while this one runs
	 * @return the decorated command
	 */
	public Command withInterruptBehavior(InterruptionBehavior behavior) {
		Command wrapped = wrap(this);
		wrapped.m_priority = behavior == InterruptionBehavior.CANCEL_INCOMING
				? CANCEL_INCOMING_PRIORITY
				: DEFAULT_PRIORITY;
		return wrapped;
	}

	/** @return this command's interruption behavior */
	public InterruptionBehavior getInterruptionBehavior() {
		return m_priority > DEFAULT_PRIORITY ? InterruptionBehavior.CANCEL_INCOMING : InterruptionBehavior.CANCEL_SELF;
	}

	// ---- Commands v3 implementation ----

	@Override
	public void run(Coroutine coroutine) {
		m_started = true;
		initialize();
		while (true) {
			execute();
			if (isFinished()) {
				break;
			}
			coroutine.yield();
		}
		m_started = false;
		end(false);
	}

	@Override
	public void onCancel() {
		// The v3 scheduler calls this when the command is canceled while running. The coroutine
		// above is abandoned at its yield(), so end(false) is never reached.
		if (m_started) {
			m_started = false;
			end(true);
		}
	}

	@Override
	public final String name() {
		return m_name;
	}

	@Override
	public final Set<Mechanism> requirements() {
		return Collections.unmodifiableSet(m_requirements);
	}

	@Override
	public final int priority() {
		return m_priority;
	}

	/** Lets {@link WrapperCommand} keep the priority of the command it wraps. */
	final void setPriority(int priority) {
		m_priority = priority;
	}

	@Override
	public String toString() {
		return m_name;
	}

	/**
	 * Presents any v3 command as a compat {@link Command}, so the v2-style decorators can be chained
	 * onto it. Used by {@link #withTimeout(double)}, {@link #withName(String)}, and the factories in
	 * this package.
	 *
	 * @param command the v3 command
	 * @return a compat command that runs {@code command}
	 */
	static Command wrap(org.wpilib.command3.Command command) {
		return new WrapperCommand(command);
	}
}
