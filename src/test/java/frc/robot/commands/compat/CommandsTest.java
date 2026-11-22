package frc.robot.commands.compat;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Mechanism;

/** Checks that the compat {@link Commands} factories behave like their Commands v2 versions. */
class CommandsTest extends CompatTestBase {
	private Mechanism m_a;
	private Mechanism m_b;

	@BeforeEach
	void createMechanisms() {
		m_a = mechanism("A");
		m_b = mechanism("B");
	}

	/** Runs loops, recording each loop's events separately. */
	private List<List<String>> trace(int loopCount) {
		List<List<String>> perLoop = new ArrayList<>();
		for (int i = 0; i < loopCount; i++) {
			m_events.clear();
			loop();
			perLoop.add(List.copyOf(m_events));
		}
		return perLoop;
	}

	// ---- sequence ----

	@Test
	void sequenceRunsEachCommandsFullLifecycleInOrder() {
		Command sequence = Commands.sequence(new RecordingCommand("x", 2, m_a), new RecordingCommand("y", 2, m_b));
		m_scheduler.schedule(sequence);
		List<List<String>> trace = trace(6);
		System.out.println("sequence trace: " + trace);

		List<String> all = trace.stream().flatMap(List::stream).toList();
		assertEquals(List.of(
				"x.initialize", "x.execute", "x.isFinished", "x.execute", "x.isFinished", "x.end(false)",
				"y.initialize", "y.execute", "y.isFinished", "y.execute", "y.isFinished", "y.end(false)"), all);
		assertFalse(m_scheduler.isScheduledOrRunning(sequence));
	}

	@Test
	void sequenceStartsTheNextCommandInTheSameLoopAsV3Does() {
		// Deliberate difference from v2 (see Commands.sequence): y initializes AND executes in
		// the loop where x finishes. v2 would only initialize y there, executing it a loop later.
		m_scheduler.schedule(
				Commands.sequence(new RecordingCommand("x", 2, m_a), new RecordingCommand("y", 2, m_b)));
		assertEquals(List.of(
				List.of("x.initialize", "x.execute", "x.isFinished"),
				List.of("x.execute", "x.isFinished", "x.end(false)", "y.initialize", "y.execute", "y.isFinished"),
				List.of("y.execute", "y.isFinished", "y.end(false)"),
				List.of()), trace(4));
	}

	@Test
	void instantCommandsInASequenceAllRunInOneLoop() {
		int[] runs = { 0 };
		m_scheduler.schedule(Commands.sequence(
				Commands.runOnce(() -> runs[0]++), Commands.runOnce(() -> runs[0]++), Commands.runOnce(() -> runs[0]++)));
		loop();
		assertEquals(3, runs[0], "v2 would take three loops");
	}

	@Test
	void sequenceRequiresEveryChildsMechanisms() {
		Command sequence = Commands.sequence(new RecordingCommand("x", 1, m_a), new RecordingCommand("y", 1, m_b));
		assertEquals(Set.of(m_a, m_b), sequence.requirements());
	}

	@Test
	void sequenceHoldsLaterChildrensMechanismsFromTheStart() {
		// v2: a group owns all its children's subsystems for its whole run, so starting the sequence
		// interrupts a command on B even though B's child hasn't started yet.
		m_scheduler.schedule(new RecordingCommand("onB", -1, m_b));
		loop();
		m_scheduler.schedule(
				Commands.sequence(new RecordingCommand("x", 3, m_a), new RecordingCommand("y", 1, m_b)));
		loop();
		assertTrue(m_events.contains("onB.end(true)"));
	}

	@Test
	void interruptingASequenceInterruptsOnlyTheRunningChild() {
		Command sequence = Commands.sequence(new RecordingCommand("x", -1, m_a), new RecordingCommand("y", 1, m_b));
		m_scheduler.schedule(sequence);
		loops(2);
		m_scheduler.cancel(sequence);
		assertTrue(m_events.contains("x.end(true)"));
		assertFalse(m_events.contains("y.initialize"), "children that never started are not ended");
	}

	// ---- parallel ----

	@Test
	void parallelRunsTogetherAndEndsWhenAllHaveEnded() {
		Command parallel = Commands.parallel(new RecordingCommand("x", 1, m_a), new RecordingCommand("y", 3, m_b));
		m_scheduler.schedule(parallel);
		List<List<String>> trace = trace(5);
		System.out.println("parallel trace: " + trace);

		assertTrue(trace.get(0).containsAll(List.of("x.initialize", "y.initialize")), "both start in the first loop");
		assertEquals(1, count3(trace, "x.end(false)"));
		assertEquals(1, count3(trace, "y.end(false)"));
		assertFalse(m_scheduler.isScheduledOrRunning(parallel));
	}

	@Test
	void parallelChildrenMayNotShareAMechanism() {
		assertThrows(IllegalArgumentException.class,
				() -> Commands.parallel(new RecordingCommand("x", 1, m_a), new RecordingCommand("y", 1, m_a)));
	}

	// ---- race ----

	@Test
	void raceEndsWithTheFirstFinisherAndInterruptsTheRest() {
		Command race = Commands.race(new RecordingCommand("fast", 2, m_a), new RecordingCommand("slow", -1, m_b));
		m_scheduler.schedule(race);
		List<String> all = trace(4).stream().flatMap(List::stream).toList();
		assertEquals(1, all.stream().filter("fast.end(false)"::equals).count());
		assertEquals(1, all.stream().filter("slow.end(true)"::equals).count());
		assertFalse(m_scheduler.isScheduledOrRunning(race));
	}

	@Test
	void raceRunsEachChildExactlyOnceEvenIfOneFinishesImmediately() {
		Command race = Commands.race(new RecordingCommand("fast", 1, m_a), new RecordingCommand("slow", -1, m_b));
		m_scheduler.schedule(race);
		List<List<String>> trace = trace(3);
		System.out.println("race trace: " + trace);
		List<String> all = trace.stream().flatMap(List::stream).toList();
		assertEquals(1, all.stream().filter("fast.initialize"::equals).count(), "fast runs once");
		assertEquals(1, all.stream().filter("slow.initialize"::equals).count(), "slow runs once");
		assertEquals(1, all.stream().filter("slow.end(true)"::equals).count());
		assertFalse(m_scheduler.isScheduledOrRunning(race));
	}

	@Test
	void raceRequiresEveryChildsMechanismsAndRejectsSharedOnes() {
		assertEquals(Set.of(m_a, m_b),
				Commands.race(new RecordingCommand("x", 1, m_a), new RecordingCommand("y", 1, m_b)).requirements());
		assertThrows(IllegalArgumentException.class,
				() -> Commands.race(new RecordingCommand("x", 1, m_a), new RecordingCommand("y", 1, m_a)));
	}

	/**
	 * Pins down the WPILib 2027.0.0-alpha-8 bug that {@link Commands#race} works around. When this
	 * test starts failing, WPILib has fixed it: delete this test and build race() on
	 * {@code Command.race()} again.
	 */
	@Test
	void nativeV3RaceStillRestartsAChildThatFinishesImmediately() {
		m_scheduler.schedule(org.wpilib.command3.Command
				.race(new RecordingCommand("fast", 1, m_a), new RecordingCommand("slow", -1, m_b))
				.withAutomaticName());
		loops(3);
		assertEquals(2, count("fast.initialize"), "WPILib bug present: fast is started twice");
	}

	@Test
	void raceWithTimeoutDecoratorStillChains() {
		Command race = Commands.race(new RecordingCommand("x", -1, m_a)).withTimeout(0.1);
		m_scheduler.schedule(race);
		loops(10);
		assertEquals(1, count("x.end(true)"));
	}

	// ---- repeatingSequence ----

	@Test
	void repeatingSequenceStartsOverAndNeverEnds() {
		Command repeat = Commands.repeatingSequence(new RecordingCommand("x", 1, m_a));
		m_scheduler.schedule(repeat);
		List<List<String>> trace = trace(6);
		System.out.println("repeatingSequence trace: " + trace);

		long passes = count3(trace, "x.end(false)");
		assertTrue(passes >= 2, "repeated at least twice in 6 loops, got " + passes);
		assertTrue(trace.stream().allMatch(l -> l.stream().filter("x.initialize"::equals).count() <= 1),
				"at most one pass per loop");
		assertTrue(m_scheduler.isScheduledOrRunning(repeat));
		assertEquals(Set.of(m_a), repeat.requirements());
	}

	@Test
	void repeatingSequenceOfInstantCommandsDoesNotHang() {
		int[] runs = { 0 };
		m_scheduler.schedule(Commands.repeatingSequence(Commands.runOnce(() -> runs[0]++)));
		loops(5); // would never return if the repeat didn't yield
		assertTrue(runs[0] >= 2 && runs[0] <= 5, "runs: " + runs[0]);
	}

	// ---- runOnce / print / waitSeconds ----

	@Test
	void runOnceRunsItsActionOnceAndKeepsRequirements() {
		int[] runs = { 0 };
		Command command = Commands.runOnce(() -> runs[0]++, m_a);
		assertEquals(Set.of(m_a), command.requirements());
		m_scheduler.schedule(command);
		loops(3);
		assertEquals(1, runs[0]);
		assertFalse(m_scheduler.isScheduledOrRunning(command));
	}

	@Test
	void runOnceDoesNotRunWhileDisabledEvenWithoutRequirements() {
		// As in v2. Plain v3 would run a command with no requirements while disabled.
		int[] runs = { 0 };
		setEnabled(false);
		m_scheduler.schedule(Commands.runOnce(() -> runs[0]++));
		m_scheduler.schedule(Commands.runOnce(() -> runs[0]++, m_a));
		loops(2);
		assertEquals(0, runs[0]);

		setEnabled(true);
		m_scheduler.schedule(Commands.runOnce(() -> runs[0]++));
		loop();
		assertEquals(1, runs[0], "runs again once enabled");
	}

	@Test
	void printPrintsTheMessage() {
		PrintStream original = System.out;
		ByteArrayOutputStream out = new ByteArrayOutputStream();
		System.setOut(new PrintStream(out, true));
		try {
			m_scheduler.schedule(Commands.print("hello from compat"));
			loop();
		} finally {
			System.setOut(original);
		}
		assertTrue(out.toString().contains("hello from compat"));
	}

	@Test
	void waitSecondsEndsAfterTheDelay() {
		Command waitCommand = Commands.waitSeconds(0.1);
		m_scheduler.schedule(waitCommand);
		loops(4); // 80 ms
		assertTrue(m_scheduler.isScheduledOrRunning(waitCommand), "still waiting at 80 ms");
		loops(3); // 140 ms
		assertFalse(m_scheduler.isScheduledOrRunning(waitCommand), "done by 140 ms");
	}

	@Test
	void factoriesAcceptNativeV3Commands() {
		int[] runs = { 0 };
		org.wpilib.command3.Command nativeCommand = org.wpilib.command3.Command
				.noRequirements(co -> runs[0]++).named("native");
		m_scheduler.schedule(Commands.sequence(nativeCommand, new RecordingCommand("x", 1, m_a)));
		loops(3);
		assertEquals(1, runs[0]);
		assertEquals(1, count("x.end(false)"));
	}

	private static long count3(List<List<String>> trace, String event) {
		return trace.stream().flatMap(List::stream).filter(event::equals).count();
	}
}
