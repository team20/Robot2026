package frc.robot.commands.compat;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

/** Checks that compat {@link SubsystemBase} behaves like a Commands v2 subsystem on v3. */
class SubsystemBaseTest extends CompatTestBase {

	@Test
	void periodicRunsOncePerLoop() {
		new RecordingSubsystem("s");
		loops(3);
		assertEquals(3, count("s.periodic"));
	}

	@Test
	void periodicRunsWhileDisabled() {
		new RecordingSubsystem("s");
		setEnabled(false);
		loops(3);
		assertEquals(3, count("s.periodic"));
	}

	@Test
	void periodicRunsBeforeCommandsInTheSameLoop() {
		RecordingSubsystem subsystem = new RecordingSubsystem("s");
		m_scheduler.schedule(new RecordingCommand("a", -1, subsystem));
		loop();
		assertEquals(List.of("s.periodic", "a.initialize", "a.execute", "a.isFinished"), m_events);
	}

	@Test
	void defaultCommandRunsWhenNothingElseUsesTheSubsystem() {
		RecordingSubsystem subsystem = new RecordingSubsystem("s");
		RecordingCommand defaultCommand = new RecordingCommand("default", -1, subsystem);
		subsystem.setDefaultCommand(defaultCommand);
		assertSame(defaultCommand, subsystem.getDefaultCommand());

		loop();
		assertEquals(List.of("s.periodic", "default.initialize", "default.execute", "default.isFinished"),
				m_events);
	}

	@Test
	void otherCommandInterruptsTheDefaultCommandWhichThenResumes() {
		RecordingSubsystem subsystem = new RecordingSubsystem("s");
		subsystem.setDefaultCommand(new RecordingCommand("default", -1, subsystem));
		loop();

		m_events.clear();
		m_scheduler.schedule(new RecordingCommand("a", 2, subsystem));
		loop();
		assertEquals(List.of("s.periodic", "default.end(true)", "a.initialize", "a.execute", "a.isFinished"),
				m_events, "the scheduled command interrupts the default command in the same loop");

		m_events.clear();
		loop(); // a finishes
		loop(); // subsystem is free again
		assertEquals(1, count("a.end(false)"));
		assertEquals(1, count("default.initialize"), "the default command restarts once the subsystem is free");
	}

	@Test
	void defaultCommandMustRequireOnlyThisSubsystem() {
		// v3 rule that v2 didn't enforce; the robot's two default commands already comply
		RecordingSubsystem subsystem = new RecordingSubsystem("s");
		RecordingSubsystem other = new RecordingSubsystem("other");
		assertThrows(IllegalArgumentException.class,
				() -> subsystem.setDefaultCommand(new RecordingCommand("both", -1, subsystem, other)));
		assertThrows(IllegalArgumentException.class,
				() -> subsystem.setDefaultCommand(new RecordingCommand("neither", -1, other)));
	}

	@Test
	void commandsRequiringASubsystemDoNotRunWhileDisabled() {
		RecordingSubsystem subsystem = new RecordingSubsystem("s");
		setEnabled(false);
		m_scheduler.schedule(new RecordingCommand("a", -1, subsystem));
		loops(2);
		assertEquals(0, count("a.initialize"));
	}

	@Test
	void disablingCancelsRunningCommandsWithEndTrue() {
		RecordingSubsystem subsystem = new RecordingSubsystem("s");
		RecordingCommand command = new RecordingCommand("a", -1, subsystem);
		m_scheduler.schedule(command);
		loop();

		setEnabled(false);
		loop();
		assertTrue(m_events.contains("a.end(true)"));
		assertFalse(m_scheduler.isScheduledOrRunning(command));
	}

	@Test
	void nameDefaultsToTheClassNameAndCanBeChanged() {
		RecordingSubsystem subsystem = new RecordingSubsystem("s");
		assertEquals("RecordingSubsystem", subsystem.getName());
		subsystem.setName("Hood");
		assertEquals("Hood", subsystem.getName());
	}

	@Test
	void registeredWithTheSchedulerItWasGiven() {
		assertSame(m_scheduler, new RecordingSubsystem("s").getRegisteredScheduler());
	}
}
