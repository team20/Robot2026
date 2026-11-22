package frc.robot.commands.compat;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

/**
 * A Commands v2-style subsystem that is a Commands v3 {@link Mechanism}.
 *
 * <p>As in v2, {@link #periodic()} runs once per scheduler loop, before any commands run, whether
 * the robot is enabled or disabled. Commands requiring this subsystem do not run while the robot is
 * disabled, which matches v2 for commands that don't override {@code runsWhenDisabled()}.
 *
 * <p>{@link #setDefaultCommand} comes from {@link Mechanism}. v3 adds one rule v2 didn't have: the
 * default command must require only this subsystem.
 */
public abstract class SubsystemBase implements Mechanism {
	private final Scheduler m_scheduler;
	private String m_name;

	/** Creates a subsystem registered with the default scheduler, as v2's constructor did. */
	protected SubsystemBase() {
		this(Scheduler.getDefault());
	}

	/**
	 * Creates a subsystem registered with a specific scheduler. Used by tests, which each run their
	 * own scheduler.
	 *
	 * @param scheduler the scheduler that runs this subsystem's periodic() and commands
	 */
	protected SubsystemBase(Scheduler scheduler) {
		m_scheduler = scheduler;
		String name = getClass().getSimpleName();
		m_name = name.isEmpty() ? "Subsystem" : name;
		// v2 registered every subsystem with the scheduler on construction so periodic() ran each
		// loop. The lambda defers the call, so overrides in subclasses are used once they exist.
		scheduler.addPeriodic(() -> periodic());
	}

	/** Called once per scheduler loop, before commands run, enabled or disabled. */
	public void periodic() {
	}

	@Override
	public final Scheduler getRegisteredScheduler() {
		return m_scheduler;
	}

	@Override
	public String getName() {
		return m_name;
	}

	public void setName(String name) {
		m_name = name;
	}

	@Override
	public String toString() {
		return m_name;
	}
}
