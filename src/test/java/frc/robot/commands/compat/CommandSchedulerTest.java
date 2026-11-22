package frc.robot.commands.compat;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.simulation.SimHooks;
import org.wpilib.telemetry.Telemetry;

/** Checks that compat {@link CommandScheduler} behaves like v2's CommandScheduler. */
class CommandSchedulerTest extends CompatTestBase {
	private CommandScheduler m_compat;
	private Mechanism m_a;
	private Mechanism m_b;

	@BeforeEach
	void createScheduler() {
		m_compat = new CommandScheduler(m_scheduler);
		m_a = mechanism("A");
		m_b = mechanism("B");
	}

	/** One robot loop through the compat scheduler, as Robot.robotPeriodic() will do. */
	private void compatLoop() {
		SimHooks.stepTiming(0.02);
		m_compat.run();
	}

	@Test
	void scheduledCommandRunsOnTheNextRun() {
		m_compat.schedule(new RecordingCommand("x", -1, m_a));
		compatLoop();
		assertEquals(1, count("x.initialize"));
		assertEquals(1, count("x.execute"));
	}

	@Test
	void scheduleAcceptsSeveralCommands() {
		m_compat.schedule(new RecordingCommand("x", -1, m_a), new RecordingCommand("y", -1, m_b));
		compatLoop();
		assertEquals(1, count("x.initialize"));
		assertEquals(1, count("y.initialize"));
	}

	@Test
	void commandScheduledFromInsideAnotherCommandOutlivesIt() {
		// The Robot.java pattern: a runOnce (ends immediately) schedules a longer command.
		RecordingCommand reset = new RecordingCommand("reset", 3, m_b);
		m_compat.schedule(Commands.runOnce(() -> m_compat.schedule(reset)));
		compatLoop(); // runOnce runs and ends; reset is handed to v3 after the run
		assertEquals(0, count("reset.initialize"), "starts on the next loop");
		assertTrue(m_compat.isScheduled(reset));

		compatLoop();
		compatLoop();
		compatLoop();
		assertEquals(1, count("reset.initialize"));
		assertEquals(1, count("reset.end(false)"), "ran to completion");
		assertEquals(0, count("reset.end(true)"), "was not canceled when the runOnce ended");
	}

	/**
	 * Why the deferral exists: scheduling straight on the v3 scheduler from inside a command makes
	 * the new command a child, which is canceled as soon as the parent ends.
	 */
	@Test
	void plainV3CancelsACommandScheduledFromInsideAnotherWhenTheParentEnds() {
		RecordingCommand reset = new RecordingCommand("reset", 3, m_b);
		m_scheduler.schedule(Commands.runOnce(() -> m_scheduler.schedule(reset)));
		loops(4);
		assertEquals(0, count("reset.end(false)"), "never completes under plain v3");
	}

	@Test
	void scheduleInterruptsAConflictingCommand() {
		m_compat.schedule(new RecordingCommand("x", -1, m_a));
		compatLoop();
		m_compat.schedule(new RecordingCommand("y", -1, m_a));
		compatLoop();
		assertEquals(1, count("x.end(true)"));
		assertEquals(1, count("y.initialize"));
	}

	@Test
	void cancelAllEndsRunningCommandsAndDropsWaitingOnes() {
		RecordingCommand waiting = new RecordingCommand("waiting", -1, m_b);
		m_compat.schedule(new RecordingCommand("x", -1, m_a));
		m_compat.schedule(Commands.runOnce(() -> m_compat.schedule(waiting)));
		compatLoop();
		assertTrue(m_compat.isScheduled(waiting));

		m_compat.cancelAll();
		assertEquals(1, count("x.end(true)"));
		assertFalse(m_compat.isScheduled(waiting));
		compatLoop();
		assertEquals(0, count("waiting.initialize"));
	}

	@Test
	void cancelEndsARunningCommand() {
		RecordingCommand command = new RecordingCommand("x", -1, m_a);
		m_compat.schedule(command);
		compatLoop();
		m_compat.cancel(command);
		assertEquals(1, count("x.end(true)"));
		assertFalse(m_compat.isScheduled(command));
	}

	@Test
	void defaultCommandsRestartAfterCancelAll() {
		RecordingSubsystem subsystem = new RecordingSubsystem("s");
		subsystem.setDefaultCommand(new RecordingCommand("default", -1, subsystem));
		compatLoop();
		m_compat.cancelAll();
		compatLoop();
		assertEquals(2, count("default.initialize"));
	}

	@Test
	void telemetryListsOnlyTopLevelCommandsLikeV2() {
		// The shape of IntakeCommands.getArmOutCombinedCommand() (right shoulder button)
		Mechanism extra = mechanism("Extra");
		Mechanism wheels = mechanism("Wheels");
		Command armOut = Commands.sequence(
				Commands.race(
						new RecordingCommand("move", 3, m_a).withTimeout(2),
						new RecordingCommand("extra", -1, extra),
						new RecordingCommand("spin", -1, wheels)),
				new RecordingCommand("spin2", -1, wheels));
		Command other = new RecordingCommand("other", -1, m_b);
		m_compat.schedule(armOut, other);
		compatLoop();

		assertTrue(m_scheduler.getRunningCommands().size() > 5, "v3 itself lists every nested child");
		assertEquals(java.util.Set.of(armOut, other), java.util.Set.copyOf(m_compat.getTopLevelCommands()),
				"one entry per scheduled command, as in v2");
	}

	@Test
	void commandTreeShowsEveryChildIndentedUnderItsParent() {
		Mechanism extra = mechanism("Extra");
		Mechanism wheels = mechanism("Wheels");
		Command armOut = Commands.sequence(
				Commands.race(
						new RecordingCommand("move", 3, m_a).withName("Move arm").withTimeout(2),
						new RecordingCommand("extra", -1, extra).withName("Spin extra"),
						new RecordingCommand("spin", -1, wheels).withName("Spin wheels")),
				new RecordingCommand("spin2", -1, wheels).withName("Spin wheels"))
				.withName("Arm out");
		m_compat.schedule(armOut);
		compatLoop();
		// withName labels stay visible, with the renamed command's class below them; wrappers that
		// would only repeat a name are folded away
		assertEquals(java.util.List.of(
				"Arm out",
				"  (Move arm [2.0 s timeout] | Spin extra | Spin wheels) -> Spin wheels",
				"    (Move arm [2.0 s timeout] | Spin extra | Spin wheels)",
				"      Move arm [2.0 s timeout]",
				"        (Move arm | Wait 2.0 s)",
				"          Move arm",
				"            RecordingCommand",
				"          Wait 2.0 s",
				"      Spin extra",
				"        RecordingCommand",
				"      Spin wheels",
				"        RecordingCommand"), m_compat.getCommandTree());

		compatLoop();
		compatLoop();
		compatLoop(); // move finishes, ending the race; the sequence moves on
		assertEquals(java.util.List.of(
				"Arm out",
				"  (Move arm [2.0 s timeout] | Spin extra | Spin wheels) -> Spin wheels",
				"    Spin wheels",
				"      RecordingCommand"), m_compat.getCommandTree());
	}

	@Test
	void telemetryListsDefaultCommands() {
		RecordingSubsystem subsystem = new RecordingSubsystem("s");
		RecordingCommand defaultCommand = new RecordingCommand("default", -1, subsystem);
		subsystem.setDefaultCommand(defaultCommand);
		compatLoop();
		assertEquals(java.util.List.of(defaultCommand), m_compat.getTopLevelCommands());
	}

	@Test
	void logsToTelemetryWithoutErrors() {
		m_compat.schedule(new RecordingCommand("x", -1, m_a));
		compatLoop();
		assertEquals("Scheduler", m_compat.getTelemetryType());
		assertDoesNotThrow(() -> Telemetry.log("CompatSchedulerTest", m_compat));
	}

	@Test
	void getInstanceIsASingletonOnTheDefaultScheduler() {
		assertSame(CommandScheduler.getInstance(), CommandScheduler.getInstance());
		assertSame(Scheduler.getDefault(), CommandScheduler.getInstance().getScheduler());
	}
}
