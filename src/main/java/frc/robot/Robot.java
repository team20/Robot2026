// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.USBSerialSubsystem;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	public static final boolean compControls = true;
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
			Constants.ControllerConstants.kOperatorControllerPort);

	{ // Here are the individual subsystems
		// new Vision();
		new USBSerialSubsystem();
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

		m_driverController.L1()
				.onTrue(new USBSerialSubsystem.SetLightCommand(USBSerialSubsystem.LightPattern.DEFAULT_GREEN));
		m_driverController.L2()
				.onTrue(new USBSerialSubsystem.SetLightCommand(USBSerialSubsystem.LightPattern.TURRET_MANUAL));
		m_driverController.R1()
				.onTrue(new USBSerialSubsystem.SetLightCommand(USBSerialSubsystem.LightPattern.ENDGAME_SHIFT));
		m_driverController.R2()
				.onTrue(new USBSerialSubsystem.SetLightCommand(USBSerialSubsystem.LightPattern.HUB_SHIFT));

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
