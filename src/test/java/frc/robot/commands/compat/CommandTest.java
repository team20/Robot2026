package frc.robot.commands.compat;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Mechanism;

/** Checks that compat {@link Command} reproduces the Commands v2 lifecycle on the v3 scheduler. */
class CommandTest extends CompatTestBase {
	private Mechanism m_mechanism;

	@BeforeEach
	void createMechanism() {
		m_mechanism = mechanism("Mechanism");
	}

	@Test
	void lifecycleMatchesV2() {
		m_scheduler.schedule(new RecordingCommand("a", 2, m_mechanism));

		loop();
		assertEquals(List.of("a.initialize", "a.execute", "a.isFinished"), m_events,
				"first loop: initialize, then execute before the first isFinished");

		m_events.clear();
		loop();
		assertEquals(List.of("a.execute", "a.isFinished", "a.end(false)"), m_events,
				"one execute per loop, then end(false) when finished");

		m_events.clear();
		loop();
		assertEquals(List.of(), m_events, "nothing runs after the command ends");
	}

	@Test
	void commandThatFinishesImmediatelyStillExecutesOnce() {
		m_scheduler.schedule(new RecordingCommand("a", 0, m_mechanism));
		loop();
		assertEquals(List.of("a.initialize", "a.execute", "a.isFinished", "a.end(false)"), m_events);
	}

	@Test
	void cancelCallsEndTrue() {
		RecordingCommand command = new RecordingCommand("a", -1, m_mechanism);
		m_scheduler.schedule(command);
		loops(2);

		m_events.clear();
		m_scheduler.cancel(command);
		assertEquals(List.of("a.end(true)"), m_events);

		m_events.clear();
		loop();
		assertEquals(List.of(), m_events, "a canceled command does not run again");
		assertFalse(m_scheduler.isScheduledOrRunning(command));
	}

	@Test
	void cancelBeforeStartingDoesNotCallEnd() {
		RecordingCommand command = new RecordingCommand("a", -1, m_mechanism);
		m_scheduler.schedule(command);
		m_scheduler.cancel(command);
		loop();
		assertEquals(List.of(), m_events, "v2 never calls end() without a matching initialize()");
	}

	@Test
	void conflictingCommandInterruptsTheRunningOne() {
		m_scheduler.schedule(new RecordingCommand("a", -1, m_mechanism));
		loop();

		m_events.clear();
		m_scheduler.schedule(new RecordingCommand("b", -1, m_mechanism));
		loop();
		assertEquals(List.of("a.end(true)", "b.initialize", "b.execute", "b.isFinished"), m_events);
	}

	@Test
	void commandsWithDifferentRequirementsRunTogether() {
		m_scheduler.schedule(new RecordingCommand("a", -1, m_mechanism));
		m_scheduler.schedule(new RecordingCommand("b", -1, mechanism("Other")));
		loop();
		assertTrue(m_events.contains("a.execute"));
		assertTrue(m_events.contains("b.execute"));
		assertFalse(m_events.stream().anyMatch(e -> e.endsWith("end(true)")));
	}

	@Test
	void sameInstanceCanBeScheduledAgainAfterItEnds() {
		RecordingCommand command = new RecordingCommand("a", 1, m_mechanism);
		m_scheduler.schedule(command);
		loop();
		m_scheduler.schedule(command);
		loop();
		assertEquals(2, count("a.initialize"));
		assertEquals(2, count("a.end(false)"));
	}

	@Test
	void withTimeoutInterruptsAfterTheTimeout() {
		RecordingCommand command = new RecordingCommand("a", -1, m_mechanism);
		Command timed = command.withTimeout(0.1);
		assertEquals(Set.of(m_mechanism), timed.requirements(), "the timeout keeps the requirements");

		m_scheduler.schedule(timed);
		loops(4); // 80 ms
		assertFalse(m_events.contains("a.end(true)"), "not timed out yet at 80 ms");

		loops(3); // 140 ms
		assertTrue(m_events.contains("a.end(true)"), "timed out by 140 ms");
		assertFalse(m_events.contains("a.end(false)"));
		assertFalse(m_scheduler.isScheduledOrRunning(timed));
	}

	@Test
	void withTimeoutRunsACommandThatFinishesImmediatelyOnlyOnce() {
		// Guards against the v3 race bug (see Commands.race): v3's own withTimeout would run it twice
		m_scheduler.schedule(new RecordingCommand("a", 1, m_mechanism).withTimeout(1.0));
		loops(3);
		assertEquals(1, count("a.initialize"));
		assertEquals(1, count("a.end(false)"));
	}

	@Test
	void withTimeoutLetsAFastCommandFinishNormally() {
		m_scheduler.schedule(new RecordingCommand("a", 2, m_mechanism).withTimeout(1.0));
		loops(2);
		assertTrue(m_events.contains("a.end(false)"));
		assertFalse(m_events.contains("a.end(true)"));
	}

	@Test
	void withNameRenamesButKeepsBehaviorAndRequirements() {
		RecordingCommand command = new RecordingCommand("a", 1, m_mechanism);
		Command named = command.withName("Renamed");

		assertEquals("Renamed", named.getName());
		assertEquals("Renamed", named.name());
		assertEquals("RecordingCommand", command.getName(), "the original keeps its own name");
		assertEquals(Set.of(m_mechanism), named.requirements());

		m_scheduler.schedule(named);
		loop();
		assertEquals(List.of("a.initialize", "a.execute", "a.isFinished", "a.end(false)"), m_events);
	}

	@Test
	void canceledWrapperInterruptsTheInnerCommand() {
		Command named = new RecordingCommand("a", -1, m_mechanism).withName("Renamed");
		m_scheduler.schedule(named);
		loop();
		m_scheduler.cancel(named);
		assertTrue(m_events.contains("a.end(true)"));
	}

	@Test
	void cancelIncomingCommandRefusesConflictingCommands() {
		// Robot.java: the hood-down command overrides all conflicting commands
		Command protectedCommand = new RecordingCommand("protected", -1, m_mechanism)
				.withInterruptBehavior(Command.InterruptionBehavior.CANCEL_INCOMING);
		assertEquals(Command.InterruptionBehavior.CANCEL_INCOMING, protectedCommand.getInterruptionBehavior());
		m_scheduler.schedule(protectedCommand);
		loop();

		m_scheduler.schedule(new RecordingCommand("incoming", -1, m_mechanism));
		loop();
		assertEquals(0, count("protected.end(true)"), "keeps running");
		assertEquals(0, count("incoming.initialize"), "the incoming command is refused");
	}

	@Test
	void cancelIncomingSurvivesFurtherDecorators() {
		Command decorated = new RecordingCommand("protected", -1, m_mechanism)
				.withInterruptBehavior(Command.InterruptionBehavior.CANCEL_INCOMING)
				.withName("Hood down")
				.withTimeout(5);
		m_scheduler.schedule(decorated);
		loop();
		m_scheduler.schedule(new RecordingCommand("incoming", -1, m_mechanism));
		loop();
		assertEquals(0, count("incoming.initialize"));
	}

	@Test
	void cancelSelfIsTheDefault() {
		RecordingCommand command = new RecordingCommand("a", -1, m_mechanism);
		assertEquals(Command.InterruptionBehavior.CANCEL_SELF, command.getInterruptionBehavior());
		assertEquals(Command.InterruptionBehavior.CANCEL_SELF,
				command.withInterruptBehavior(Command.InterruptionBehavior.CANCEL_INCOMING)
						.withInterruptBehavior(Command.InterruptionBehavior.CANCEL_SELF)
						.getInterruptionBehavior());
	}

	@Test
	void cancelIncomingCommandCanStillBeCanceledDirectly() {
		Command protectedCommand = new RecordingCommand("protected", -1, m_mechanism)
				.withInterruptBehavior(Command.InterruptionBehavior.CANCEL_INCOMING);
		m_scheduler.schedule(protectedCommand);
		loop();
		m_scheduler.cancel(protectedCommand);
		assertEquals(1, count("protected.end(true)"), "e.g. when the trigger is released");
	}

	@Test
	void defaultNameIsTheSimpleClassName() {
		assertEquals("RecordingCommand", new RecordingCommand("a", 1).getName());
	}

	@Test
	void addRequirementsIgnoresNullAndDuplicates() {
		RecordingCommand command = new RecordingCommand("a", 1, m_mechanism, null, m_mechanism);
		assertEquals(Set.of(m_mechanism), command.requirements());
	}
}
