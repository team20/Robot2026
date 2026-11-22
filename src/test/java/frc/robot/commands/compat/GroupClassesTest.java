package frc.robot.commands.compat;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Mechanism;

/**
 * Checks the v2-named classes the robot code creates with {@code new}: SequentialCommandGroup,
 * ParallelRaceGroup, WaitCommand, and RepeatCommand.
 */
class GroupClassesTest extends CompatTestBase {
	private Mechanism m_a;
	private Mechanism m_b;

	@BeforeEach
	void createMechanisms() {
		m_a = mechanism("A");
		m_b = mechanism("B");
	}

	@Test
	void sequentialCommandGroupRunsInOrderWithCombinedRequirements() {
		Command group = new SequentialCommandGroup(new RecordingCommand("x", 1, m_a), new RecordingCommand("y", 1, m_b));
		assertEquals(Set.of(m_a, m_b), group.requirements());
		m_scheduler.schedule(group);
		loops(3);
		assertEquals(List.of(
				"x.initialize", "x.execute", "x.isFinished", "x.end(false)",
				"y.initialize", "y.execute", "y.isFinished", "y.end(false)"), m_events);
		assertFalse(m_scheduler.isScheduledOrRunning(group));
	}

	@Test
	void parallelRaceGroupEndsWithTheFirstFinisher() {
		Command race = new ParallelRaceGroup(new RecordingCommand("fast", 1, m_a), new RecordingCommand("slow", -1, m_b));
		m_scheduler.schedule(race);
		loops(3);
		assertEquals(1, count("fast.initialize"), "each child runs once");
		assertEquals(1, count("fast.end(false)"));
		assertEquals(1, count("slow.end(true)"));
		assertFalse(m_scheduler.isScheduledOrRunning(race));
	}

	@Test
	void waitCommandEndsAfterTheDelay() {
		Command wait = new WaitCommand(0.1);
		m_scheduler.schedule(wait);
		loops(4); // 80 ms
		assertTrue(m_scheduler.isScheduledOrRunning(wait), "still waiting at 80 ms");
		loops(3); // 140 ms
		assertFalse(m_scheduler.isScheduledOrRunning(wait), "done by 140 ms");
	}

	@Test
	void repeatCommandRestartsItsCommandOncePerLoopAndNeverEnds() {
		Command repeat = new RepeatCommand(new RecordingCommand("x", 1, m_a));
		assertEquals(Set.of(m_a), repeat.requirements());
		m_scheduler.schedule(repeat);
		loops(4);
		assertEquals(4, count("x.initialize"));
		assertEquals(4, count("x.end(false)"));
		assertTrue(m_scheduler.isScheduledOrRunning(repeat));
	}

	@Test
	void canceledRepeatCommandInterruptsItsCommand() {
		Command repeat = new RepeatCommand(new RecordingCommand("x", -1, m_a));
		m_scheduler.schedule(repeat);
		loops(2);
		m_scheduler.cancel(repeat);
		assertEquals(1, count("x.end(true)"));
	}

	@Test
	void autoComposerPatternRepeatRacedInsideASequence() {
		// The shape used in AutoComposer: aim repeatedly while driving, then shoot.
		//   new SequentialCommandGroup(
		//       new ParallelRaceGroup(new RepeatCommand(aim), drive),
		//       shoot)
		Command auto = new SequentialCommandGroup(
				new ParallelRaceGroup(
						new RepeatCommand(new RecordingCommand("aim", 1, m_a)),
						new RecordingCommand("drive", 3, m_b)),
				new RecordingCommand("shoot", 1, m_a));
		assertEquals(Set.of(m_a, m_b), auto.requirements());

		m_scheduler.schedule(auto);
		loops(5);
		assertEquals(1, count("drive.end(false)"), "drive finishes, ending the race");
		assertTrue(count("aim.initialize") >= 2, "aim repeated while driving");
		assertEquals(0, count("aim.end(true)"), "aim's last pass had already finished, so nothing to interrupt");
		assertEquals(1, count("shoot.end(false)"), "shoot runs after the race");
		assertTrue(m_events.indexOf("shoot.initialize") > m_events.indexOf("drive.end(false)"));
		assertFalse(m_scheduler.isScheduledOrRunning(auto));
	}

	@Test
	void robotPatternWaitCommandsInASequence() {
		// The shape used in Robot.teleopInit: wait, then do something, repeated.
		Command sequence = new SequentialCommandGroup(new WaitCommand(0.1), new RecordingCommand("x", 1, m_a));
		m_scheduler.schedule(sequence);
		loops(4); // 80 ms
		assertEquals(0, count("x.initialize"), "x waits for the WaitCommand");
		loops(3); // 140 ms
		assertEquals(1, count("x.end(false)"));
	}

	@Test
	void groupClassesAreCompatCommandsSoDecoratorsChain() {
		Command timed = new SequentialCommandGroup(new RecordingCommand("x", -1, m_a)).withTimeout(0.1);
		m_scheduler.schedule(timed);
		loops(10);
		assertEquals(1, count("x.end(true)"));
		assertEquals("Auto", new WaitCommand(2).withName("Auto").getName());
	}
}
