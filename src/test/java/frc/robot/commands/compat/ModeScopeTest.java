package frc.robot.commands.compat;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Pins down v3's robot-mode scoping, which v2 doesn't have: button bindings and default commands
 * created while the robot is in a mode (Robot.teleopInit() creates all of them) only work while
 * the driver station stays in that mode. In v2 they worked in every mode once created.
 */
class ModeScopeTest extends CompatTestBase {
	private CommandScheduler m_compat;
	private final boolean[] m_pressed = { false };

	@BeforeEach
	void setUp() {
		// On the robot, RobotBase does this; until then the robot mode always reads UNKNOWN
		DriverStationBackend.observeUserProgramStarting();
		m_compat = new CommandScheduler(m_scheduler);
		setMode(RobotMode.TELEOPERATED);
	}

	@AfterEach
	void resetMode() {
		setMode(RobotMode.UNKNOWN);
		DriverStationBackend.clearUserProgramStarted();
	}

	private static void setMode(RobotMode mode) {
		DriverStationSim.setRobotMode(mode);
		DriverStationSim.notifyNewData();
		DriverStationBackend.refreshData();
	}

	private void compatLoop() {
		SimHooks.stepTiming(0.02);
		m_compat.run();
	}

	private void press() {
		m_pressed[0] = true;
		compatLoop();
		m_pressed[0] = false;
		compatLoop();
	}

	@Test
	void bindingMadeInTeleopWorksInTeleop() {
		new Trigger(m_scheduler, () -> m_pressed[0]).onTrue(new RecordingCommand("x", 1, mechanism("A")));
		press();
		assertEquals(1, count("x.initialize"));
	}

	@Test
	void bindingMadeInTeleopSurvivesDisablingWithTeleopStillSelected() {
		new Trigger(m_scheduler, () -> m_pressed[0]).onTrue(new RecordingCommand("x", 1, mechanism("A")));
		setEnabled(false);
		compatLoop();
		setEnabled(true);
		press();
		assertEquals(1, count("x.initialize"));
	}

	@Test
	void bindingMadeInTeleopDoesNothingInAutonomous() {
		// v2: the button would still work in autonomous. Only matters when switching from teleop to
		// autonomous without restarting robot code (practice, sim); a match runs auto first.
		new Trigger(m_scheduler, () -> m_pressed[0]).onTrue(new RecordingCommand("x", 1, mechanism("A")));
		setMode(RobotMode.AUTONOMOUS);
		compatLoop();
		press();
		assertEquals(0, count("x.initialize"));
	}

	@Test
	void bindingWorksAgainAfterTeleopInitRebindsIt() {
		// Robot.teleopInit() calls bindCompControls() every time teleop starts
		Trigger trigger = new Trigger(m_scheduler, () -> m_pressed[0]);
		RecordingCommand command = new RecordingCommand("x", 1, mechanism("A"));
		trigger.onTrue(command);
		setMode(RobotMode.AUTONOMOUS);
		compatLoop();
		setMode(RobotMode.TELEOPERATED);
		trigger.onTrue(command); // teleopInit binds again
		press();
		assertEquals(1, count("x.initialize"));
	}

	@Test
	void defaultCommandSetInTeleopDoesNotRunInAutonomous() {
		// v2: JoystickDrive and GradualAim would keep running as defaults in autonomous
		RecordingSubsystem subsystem = new RecordingSubsystem("s");
		subsystem.setDefaultCommand(new RecordingCommand("default", -1, subsystem));
		compatLoop();
		assertEquals(1, count("default.initialize"));

		setMode(RobotMode.AUTONOMOUS);
		compatLoop();
		compatLoop();
		assertEquals(1, count("default.end(true)"), "ended when the mode changed");
		assertEquals(1, count("default.initialize"), "not restarted in autonomous");
	}

	@Test
	void commandScheduledInAutonomousEndsWhenTeleopStarts() {
		// v2 kept it running, but Robot.teleopInit() calls cancelAll() anyway, so no change in practice
		setMode(RobotMode.AUTONOMOUS);
		m_compat.schedule(new RecordingCommand("auto", -1, mechanism("A")));
		compatLoop();
		setMode(RobotMode.TELEOPERATED);
		compatLoop();
		assertEquals(1, count("auto.end(true)"));
	}
}
