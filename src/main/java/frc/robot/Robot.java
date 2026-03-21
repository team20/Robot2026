// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	public static final boolean compControls = true;
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
			Constants.ControllerConstants.kOperatorControllerPort);

	{ // Here are the individual subsystems
		new Vision();
	}

	public void initSubsystems() {
	}

	public Robot() {

	}

	@Override
	public void robotPeriodic() {
		m_scheduler.run();
		if (Constants.kLogging) {
			SmartDashboard.putData(m_scheduler);
		}
	}

	@Override
	public void autonomousInit() {
		initSubsystems();
		m_scheduler.cancelAll();

	}

	@Override
	public void teleopInit() {
		initSubsystems();
		m_scheduler.cancelAll();

	}

	@Override
	public void testInit() {
		initSubsystems();
		m_scheduler.cancelAll();
		m_scheduler.schedule(Commands.sequence(ClampedP.testCommand()));
	}
}
