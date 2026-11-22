package frc.robot.commands.compat;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.command3.Scheduler;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;

/**
 * Commands v2's {@code CommandScheduler}, forwarding to the Commands v3 {@link Scheduler}.
 *
 * <p>One behavior needs special handling. In v3, a command scheduled from inside another running
 * command becomes that command's child and is canceled as soon as the parent ends. v2 had no such
 * link: for example, {@code Robot} schedules {@code ResetOdometry} from inside a
 * {@code Commands.runOnce}, which ends immediately. So {@link #schedule} called from inside a
 * running command holds the command until the end of the current {@link #run()}, then hands it to
 * v3 from outside any command. It starts on the next loop. Called from anywhere else (robot init
 * methods, mode changes), {@code schedule} hands the command to v3 immediately.
 *
 * <p>Logging this object with {@code Telemetry.log("Scheduler", ...)} publishes:
 * <ul>
 * <li>{@code Names}, {@code Ids}: the commands that were scheduled directly (one per button press,
 * plus default commands), like v2's scheduler did.
 * <li>{@code All}: every running command including the children inside groups, as an indented
 * tree, for debugging.
 * </ul>
 */
public final class CommandScheduler implements TelemetryLoggable {
	private static CommandScheduler s_instance;

	private final Scheduler m_scheduler;
	// Commands scheduled from inside a running command, waiting for the end of run()
	private final List<org.wpilib.command3.Command> m_deferred = new ArrayList<>();

	/** @return the scheduler the robot uses, backed by {@link Scheduler#getDefault()} */
	public static synchronized CommandScheduler getInstance() {
		if (s_instance == null) {
			s_instance = new CommandScheduler(Scheduler.getDefault());
		}
		return s_instance;
	}

	/** For tests, which each run their own v3 scheduler. */
	CommandScheduler(Scheduler scheduler) {
		m_scheduler = scheduler;
	}

	/** @return the Commands v3 scheduler this forwards to */
	public Scheduler getScheduler() {
		return m_scheduler;
	}

	/** Runs one scheduler loop. Call once per robot loop, from {@code robotPeriodic()}. */
	public void run() {
		m_scheduler.run();
		// Now outside every command: hand over commands that were scheduled from inside one
		List<org.wpilib.command3.Command> deferred = new ArrayList<>(m_deferred);
		m_deferred.clear();
		for (org.wpilib.command3.Command command : deferred) {
			m_scheduler.schedule(command);
		}
	}

	/**
	 * Schedules commands. A command that uses a mechanism another command is using interrupts that
	 * command, as in v2.
	 *
	 * @param commands the commands to schedule
	 */
	public void schedule(org.wpilib.command3.Command... commands) {
		boolean insideACommand = m_scheduler.currentCommand() != null;
		for (org.wpilib.command3.Command command : commands) {
			if (insideACommand) {
				m_deferred.add(command);
			} else {
				m_scheduler.schedule(command);
			}
		}
	}

	/**
	 * Cancels commands, calling {@code end(true)} on compat commands that had started.
	 *
	 * @param commands the commands to cancel
	 */
	public void cancel(org.wpilib.command3.Command... commands) {
		for (org.wpilib.command3.Command command : commands) {
			m_deferred.remove(command);
			m_scheduler.cancel(command);
		}
	}

	/** Cancels every running and waiting command. Default commands start again on the next loop. */
	public void cancelAll() {
		m_deferred.clear();
		m_scheduler.cancelAll();
	}

	/**
	 * @param command the command
	 * @return whether the command is waiting to start or running
	 */
	public boolean isScheduled(org.wpilib.command3.Command command) {
		return m_deferred.contains(command) || m_scheduler.isScheduledOrRunning(command);
	}

	/**
	 * The running commands that were scheduled directly (by a trigger, robot code, or as a default
	 * command), leaving out the children running inside groups and decorators. This is what v2's
	 * scheduler listed. v3's {@link Scheduler#getRunningCommands()} also lists every child, so one
	 * button press running a sequence of races with timeouts shows up as a dozen entries.
	 *
	 * @return the top-level running commands
	 */
	List<org.wpilib.command3.Command> getTopLevelCommands() {
		return m_scheduler.getRunningCommands().stream()
				.filter(command -> m_scheduler.getParentOf(command) == null)
				.toList();
	}

	/**
	 * Every running command, including the children inside groups and decorators, as an indented
	 * tree: each child appears below its parent, indented two spaces further. Compat wrappers (from
	 * the {@link Commands} factories and the group classes) usually have the same name as the
	 * command they wrap; those are folded into it rather than listed twice. Wrappers with their own
	 * name ({@code withName}, {@code withTimeout}) are listed, with the wrapped command below them.
	 * For debugging; {@link #getTopLevelCommands()} is the v2-style list.
	 *
	 * @return one line per running command
	 */
	List<String> getCommandTree() {
		var running = m_scheduler.getRunningCommands();
		List<String> lines = new ArrayList<>();
		for (var command : running) {
			if (m_scheduler.getParentOf(command) == null) {
				addToTree(command, 0, running, lines);
			}
		}
		return lines;
	}

	private void addToTree(org.wpilib.command3.Command command, int depth,
			java.util.Collection<org.wpilib.command3.Command> running, List<String> lines) {
		List<org.wpilib.command3.Command> children = running.stream()
				.filter(child -> m_scheduler.getParentOf(child) == command)
				.toList();
		// Fold a wrapper only when it would repeat the same name as the one command inside it.
		// A wrapper renamed by withName() or withTimeout() stays, so those names are visible.
		boolean foldWrapper = command instanceof WrapperCommand && children.size() == 1
				&& children.get(0).name().equals(command.name());
		if (!foldWrapper) {
			lines.add("  ".repeat(depth) + command.name());
		}
		int childDepth = foldWrapper ? depth : depth + 1;
		for (var child : children) {
			addToTree(child, childDepth, running, lines);
		}
	}

	@Override
	public void logTo(TelemetryTable table) {
		var running = getTopLevelCommands();
		table.log("Names", running.stream().map(org.wpilib.command3.Command::name).toArray(String[]::new));
		table.log("Ids", running.stream().mapToLong(m_scheduler::runId).toArray());
		table.log("All", getCommandTree().toArray(String[]::new));
	}

	@Override
	public String getTelemetryType() {
		return "Scheduler";
	}
}
