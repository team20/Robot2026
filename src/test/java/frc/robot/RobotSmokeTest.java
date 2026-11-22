package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.lang.reflect.Method;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringSubscriber;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

import frc.robot.commands.compat.CommandScheduler;
import frc.robot.subsystems.USBSerialSubsystem;

/**
 * Builds the real Robot in simulation and runs teleop and the default auto. Catches commands,
 * groups, bindings, and default commands that Commands v3 rejects when they are created or run
 * (for example a default command requiring two subsystems, or parallel children sharing one),
 * which would otherwise only show up when the robot is enabled.
 */
class RobotSmokeTest {
	private static Robot s_robot;

	@BeforeAll
	static void createRobot() {
		HAL.initialize();
		DriverStationBackend.observeUserProgramStarting();
		s_robot = new Robot();
	}

	@AfterAll
	static void reset() {
		setMode(RobotMode.UNKNOWN, false);
		DriverStationBackend.clearUserProgramStarted();
		SimHooks.resumeTiming();
	}

	private static void setMode(RobotMode mode, boolean enabled) {
		DriverStationSim.setRobotMode(mode);
		DriverStationSim.setEnabled(enabled);
		DriverStationSim.notifyNewData();
		DriverStationBackend.refreshData();
	}

	private static void runLoops(int count) {
		SimHooks.pauseTiming();
		for (int i = 0; i < count; i++) {
			SimHooks.stepTiming(0.02);
			DriverStationBackend.refreshData();
			s_robot.robotPeriodic();
		}
	}

	@Test
	void teleopWithCompetitionAndTestBindingsRunsWithoutErrors() throws Exception {
		setMode(RobotMode.TELEOPERATED, true);
		assertDoesNotThrow(() -> s_robot.teleopInit(), "creating the competition bindings");

		// Also create the test-controls bindings (normally chosen by Robot.compControls)
		Method bindTestControls = Robot.class.getDeclaredMethod("bindTestControls");
		bindTestControls.setAccessible(true);
		assertDoesNotThrow(() -> bindTestControls.invoke(s_robot), "creating the test bindings");

		assertDoesNotThrow(() -> runLoops(100), "two seconds of teleop");
		assertFalse(CommandScheduler.getInstance().getScheduler().getRunningCommands().isEmpty(),
				"default commands (drive, turret) and the LED sequence are running");
	}

	@Test
	void defaultAutoRunsWithoutErrors() {
		setMode(RobotMode.AUTONOMOUS, true);
		assertDoesNotThrow(() -> s_robot.autonomousInit());
		assertDoesNotThrow(() -> runLoops(250), "five seconds of auto");
	}

	// ---- Auto-aim lights (USBSerialSubsystem.showAimStatus) ----

	private static final NetworkTableInstance s_nt = NetworkTableInstance.getDefault();
	private static final IntegerSubscriber s_lightSends = s_nt.getIntegerTopic("/Telemetry/Lights/Sends").subscribe(0);
	private static final StringSubscriber s_lightPattern = s_nt.getStringTopic("/Telemetry/Lights/Pattern")
			.subscribe("");

	@Test
	void aimStatusLightsAreSentOnlyWhenTheStatusChanges() {
		setMode(RobotMode.TELEOPERATED, true);
		CommandScheduler.getInstance().cancelAll();
		USBSerialSubsystem lights = USBSerialSubsystem.get();
		lights.m_isHubShift.set(false);
		boolean[] seesHub = { false };
		org.wpilib.command3.Command showAimStatus = lights.showAimStatus(() -> seesHub[0]);
		long start = s_lightSends.get();

		CommandScheduler.getInstance().schedule(showAimStatus);
		runLoops(25);
		assertEquals(1, s_lightSends.get() - start, "sends the status once when it starts");
		assertEquals("VISION_AIM_NOT_LOCKED", s_lightPattern.get());

		seesHub[0] = true;
		runLoops(25);
		assertEquals(2, s_lightSends.get() - start, "sends again only when the status changes");
		assertEquals("VISION_AIM_LOCKED", s_lightPattern.get());

		lights.m_isHubShift.set(true);
		runLoops(10);
		assertEquals(2, s_lightSends.get() - start, "stays quiet while hub-shift lights are showing");
		lights.m_isHubShift.set(false);
		runLoops(10);
		assertEquals(3, s_lightSends.get() - start, "sends the status again once the hub shift ends");
		assertEquals("VISION_AIM_LOCKED", s_lightPattern.get());

		CommandScheduler.getInstance().cancel(showAimStatus);
		runLoops(10);
		assertEquals(3, s_lightSends.get() - start, "stops when canceled");
	}

	@Test
	void autoAimButtonSendsTheLightsOncePerPress() {
		// The operator's Back button (with both triggers released) runs auto-aim. It used to re-send the
		// light byte every loop (50 per second) while held.
		setMode(RobotMode.TELEOPERATED, true);
		s_robot.teleopInit();
		USBSerialSubsystem.get().m_isHubShift.set(false);
		int operator = Constants.ControllerConstants.kOperatorControllerPort;
		DriverStationSim.setJoystickIsGamepad(operator, true);
		DriverStationSim.setJoystickButtonsMaximumIndex(operator, 20);
		DriverStationSim.setJoystickButtonsAvailable(operator, 0xFFFFFL);
		runLoops(10);
		long start = s_lightSends.get();

		pressOperatorBack(true);
		runLoops(50); // one second held
		assertEquals(1, s_lightSends.get() - start, "one send for one second of auto-aim");
		assertEquals("VISION_AIM_NOT_LOCKED", s_lightPattern.get(), "no vision in simulation, so not locked");

		pressOperatorBack(false);
		runLoops(10);
		assertEquals(1, s_lightSends.get() - start, "nothing more after release");

		pressOperatorBack(true);
		runLoops(10);
		assertEquals(2, s_lightSends.get() - start, "pressing again shows the status again");
		pressOperatorBack(false);
		runLoops(5);
	}

	private static void pressOperatorBack(boolean pressed) {
		DriverStationSim.setJoystickButton(Constants.ControllerConstants.kOperatorControllerPort,
				Gamepad.Button.BACK.value, pressed);
		DriverStationSim.notifyNewData();
	}

	@Test
	void disablingCancelsEverythingWithoutErrors() {
		setMode(RobotMode.TELEOPERATED, true);
		s_robot.teleopInit();
		runLoops(10);
		setMode(RobotMode.TELEOPERATED, false);
		assertDoesNotThrow(() -> s_robot.disabledInit());
		assertDoesNotThrow(() -> runLoops(10));
	}
}
