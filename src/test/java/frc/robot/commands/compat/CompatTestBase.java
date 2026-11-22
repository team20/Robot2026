package frc.robot.commands.compat;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Shared setup for compat tests: HAL, an enabled robot, paused simulated time, a fresh scheduler
 * per test, and an event log that {@link RecordingCommand} and {@link RecordingSubsystem} write to.
 */
abstract class CompatTestBase {
	protected Scheduler m_scheduler;
	protected final List<String> m_events = new ArrayList<>();

	@BeforeAll
	static void initHal() {
		HAL.initialize();
	}

	@BeforeEach
	void setUpScheduler() {
		setEnabled(true);
		SimHooks.pauseTiming();
		m_scheduler = Scheduler.createIndependentScheduler();
		m_events.clear();
	}

	@AfterEach
	void resumeTiming() {
		SimHooks.resumeTiming();
	}

	protected static void setEnabled(boolean enabled) {
		DriverStationSim.setEnabled(enabled);
		DriverStationSim.notifyNewData();
	}

	/** One robot loop: advance simulated time by 20 ms, then run the scheduler. */
	protected void loop() {
		SimHooks.stepTiming(0.02);
		m_scheduler.run();
	}

	protected void loops(int count) {
		for (int i = 0; i < count; i++) {
			loop();
		}
	}

	protected long count(String event) {
		return m_events.stream().filter(event::equals).count();
	}

	/** A plain v3 mechanism on this test's scheduler. */
	protected Mechanism mechanism(String name) {
		Scheduler scheduler = m_scheduler;
		return new Mechanism() {
			@Override
			public Scheduler getRegisteredScheduler() {
				return scheduler;
			}

			@Override
			public String getName() {
				return name;
			}
		};
	}

	/** A compat subsystem on this test's scheduler that records each periodic() call. */
	protected class RecordingSubsystem extends SubsystemBase {
		private final String m_label;

		RecordingSubsystem(String label) {
			super(m_scheduler);
			m_label = label;
		}

		@Override
		public void periodic() {
			m_events.add(m_label + ".periodic");
		}
	}

	/** Records every lifecycle call. Finishes after {@code executesToFinish} executes (-1 = never). */
	protected class RecordingCommand extends Command {
		private final String m_label;
		private final int m_executesToFinish;
		private int m_executes;

		RecordingCommand(String label, int executesToFinish, Mechanism... requirements) {
			m_label = label;
			m_executesToFinish = executesToFinish;
			addRequirements(requirements);
		}

		@Override
		public void initialize() {
			m_executes = 0;
			m_events.add(m_label + ".initialize");
		}

		@Override
		public void execute() {
			m_executes++;
			m_events.add(m_label + ".execute");
		}

		@Override
		public boolean isFinished() {
			m_events.add(m_label + ".isFinished");
			return m_executesToFinish >= 0 && m_executes >= m_executesToFinish;
		}

		@Override
		public void end(boolean interrupted) {
			m_events.add(m_label + ".end(" + interrupted + ")");
		}
	}
}
