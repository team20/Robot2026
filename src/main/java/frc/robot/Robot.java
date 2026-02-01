// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
	private final CommandPS5Controller m_joystick = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);

	public Robot() {

	}

	private void BindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_joystick.getLeftY(), () -> -m_joystick.getLeftX(),
						() -> m_joystick.getL2Axis() - m_joystick.getR2Axis(), m_joystick.getHID()::getCreateButton));
		m_joystick.options().onTrue(m_driveSubsystem.resetHeading());
	}

	@Override
	public void robotPeriodic() {
		m_scheduler.run();

		SmartDashboard.putData(m_scheduler);
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void disabledExit() {

	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = new frc.robot.commands.DriveDistance(1).withTimeout(Time.ofBaseUnits(1, Seconds));// m_autoChooser.getSelected();

		if (m_autonomousCommand != null) {
			m_scheduler.schedule(m_autonomousCommand);
		}
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		BindDriveControls();
	}

	@Override
	public void teleopPeriodic() {

	}

	@Override
	public void teleopExit() {
		m_driveSubsystem.removeDefaultCommand();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	@Override
	public void simulationPeriodic() {

	}
}
