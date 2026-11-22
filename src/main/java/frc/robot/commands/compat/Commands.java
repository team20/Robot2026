package frc.robot.commands.compat;

import static org.wpilib.units.Units.Seconds;

import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.RobotState;

/**
 * Commands v2-style factory methods, built on Commands v3. Each returns a compat {@link Command}, so
 * v2 decorators such as {@code withTimeout(double)} and {@code withName(String)} still chain. The
 * arguments may be compat commands or native v3 commands.
 *
 * <p>Groups behave as in v2: a group requires every mechanism its children require, for as long as
 * it runs, and children of a parallel group or race may not share a mechanism.
 */
public final class Commands {
	private Commands() {
	}

	/**
	 * Runs the commands one after another.
	 *
	 * <p>Timing follows Commands v3, not v2: when one command finishes, the next one's
	 * {@code initialize()} and first {@code execute()} run in that same loop. In v2 the next
	 * command's first {@code execute()} waited until the following loop, so each step of a v3
	 * sequence starts working 20 ms sooner, and several instant commands in a row all run in one
	 * loop. This was chosen deliberately so the robot code gets used to v3 timing before it is
	 * rewritten in native v3. If a step really needs a gap, add {@code waitSeconds(0.02)}.
	 *
	 * @param commands the commands, in order
	 * @return the sequence
	 */
	public static Command sequence(org.wpilib.command3.Command... commands) {
		return Command.wrap(org.wpilib.command3.Command.sequence(commands).withAutomaticName());
	}

	/**
	 * Runs the commands at the same time; ends when all of them have ended.
	 *
	 * @param commands the commands
	 * @return the parallel group
	 */
	public static Command parallel(org.wpilib.command3.Command... commands) {
		return Command.wrap(org.wpilib.command3.Command.parallel(commands).withAutomaticName());
	}

	/**
	 * Runs the commands at the same time; ends when the first one ends, interrupting the others
	 * ({@code end(true)}).
	 *
	 * @param commands the commands
	 * @return the race
	 */
	public static Command race(org.wpilib.command3.Command... commands) {
		// Workaround for a bug in WPILib 2027.0.0-alpha-8: a race made with v3's Command.race()
		// starts its children twice. A child that finishes on its first loop therefore runs a
		// second time. Instead, we run the children with coroutine.awaitAny(), which starts each
		// one exactly once and cancels the rest when the first one finishes.
		//
		// We still build v3's race below, but never run it. Building it checks that no two
		// children use the same mechanism (throwing if they do), and gives us the combined
		// requirements and a readable name.
		//
		// CommandsTest.nativeV3RaceStillRestartsAChildThatFinishesImmediately fails once WPILib
		// fixes the bug; then this method can go back to wrapping Command.race().
		org.wpilib.command3.Command validated = org.wpilib.command3.Command.race(commands)
				.withAutomaticName();
		List<org.wpilib.command3.Command> children = List.of(commands);
		return Command.wrap(withRequirements(validated.requirements(),
				coroutine -> coroutine.awaitAny(children), validated.name(), validated.priority()));
	}

	/**
	 * Runs the commands in sequence, starting over each time the last one ends. Never ends on its own.
	 *
	 * @param commands the commands, in order
	 * @return the repeating sequence
	 */
	public static Command repeatingSequence(org.wpilib.command3.Command... commands) {
		org.wpilib.command3.Command sequence = org.wpilib.command3.Command.sequence(commands)
				.withAutomaticName();
		return Command.wrap(withRequirements(sequence.requirements(), coroutine -> {
			while (true) {
				coroutine.await(sequence);
				// One pass per loop at most, as in v2; also keeps a sequence of instant commands
				// from repeating forever without ever yielding to the rest of the robot code.
				coroutine.yield();
			}
		}, "Repeat " + sequence.name(), sequence.priority()));
	}

	/**
	 * Runs an action once and ends immediately. As in v2, the action does not run while the robot is
	 * disabled.
	 *
	 * @param action the action
	 * @param requirements the mechanisms the action uses
	 * @return the command
	 */
	public static Command runOnce(Runnable action, Mechanism... requirements) {
		// v3 refuses commands that require a subsystem while disabled, but lets commands with no
		// requirements run. v2's runOnce never ran while disabled, so check here too. (Robot's
		// driver back button would otherwise reset the odometry pose while disabled.)
		return Command.wrap(withRequirements(new LinkedHashSet<>(Arrays.asList(requirements)),
				coroutine -> {
					if (RobotState.isEnabled()) {
						action.run();
					}
				}, "RunOnce"));
	}

	/**
	 * Prints a message to the console and ends immediately.
	 *
	 * @param message the message
	 * @return the command
	 */
	public static Command print(String message) {
		return Command.wrap(withRequirements(Set.of(), coroutine -> System.out.println(message),
				"Print: " + message));
	}

	/**
	 * Does nothing for a time, then ends.
	 *
	 * @param seconds how long to wait
	 * @return the command
	 */
	public static Command waitSeconds(double seconds) {
		return Command.wrap(org.wpilib.command3.Command.waitFor(Seconds.of(seconds))
				.named("Wait " + seconds + " s"));
	}

	/** Builds a named v3 command with the given requirements (which may be empty) and body. */
	private static org.wpilib.command3.Command withRequirements(Set<Mechanism> requirements,
			Consumer<Coroutine> body, String name) {
		return withRequirements(requirements, body, name, org.wpilib.command3.Command.DEFAULT_PRIORITY);
	}

	/**
	 * Same, with a priority. Groups pass their children's highest priority, as v3's own groups do,
	 * so a group containing a CANCEL_INCOMING command can't be interrupted either.
	 */
	private static org.wpilib.command3.Command withRequirements(Set<Mechanism> requirements,
			Consumer<Coroutine> body, String name, int priority) {
		if (requirements.isEmpty()) {
			return org.wpilib.command3.Command.noRequirements(body).withPriority(priority).named(name);
		}
		return org.wpilib.command3.Command.requiring(requirements).executing(body).withPriority(priority)
				.named(name);
	}
}
