// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.Turret;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private final Drive m_driveSubsystem = new Drive(PhysicalBot.DENNIS);
	private final Transport m_transportSubsystem = new Transport();
	private final Intake m_intakeSubsystem = new Intake();
	private final Shooter m_shooterSubsystem = new Shooter();
	private final Turret m_turretSubsystem = new Turret();
	private final Climber m_climberSubsystem = new Climber();
	private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
	private final CommandPS5Controller m_joystick = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);

	public Robot() {
		BindDriveControls();
	}

	private void BindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_joystick.getLeftY(), () -> -m_joystick.getLeftX(),
						() -> m_joystick.getL2Axis() - m_joystick.getR2Axis(), m_joystick.getHID()::getCreateButton));
		m_transportSubsystem.setDefaultCommand(
				m_transportSubsystem.moveWithTrigger(
						m_joystick.triangle(),
						m_joystick.cross()));
		m_intakeSubsystem.setDefaultCommand(
				m_intakeSubsystem.moveWithTrigger(
						m_joystick.R1(),
						m_joystick.L1()));
		m_joystick.circle().onTrue(
				m_intakeSubsystem.deployRollers());
		m_joystick.circle().onTrue(
				m_intakeSubsystem.retractRollers());
		m_shooterSubsystem.setDefaultCommand(
				m_shooterSubsystem.moveWithTrigger(
						m_joystick.square(), null));
		m_turretSubsystem.setDefaultCommand(
				m_turretSubsystem.aimWithJoystick(
						m_joystick::getLeftX,
						m_joystick::getLeftY));
		m_climberSubsystem.setDefaultCommand(
				m_climberSubsystem.moveWithTrigger(
						m_joystick.povUp(), m_joystick.povDown()));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
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
		m_autonomousCommand = m_autoChooser.getSelected();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
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
	}

	@Override
	public void teleopPeriodic() {

	}

	@Override
	public void teleopExit() {
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
}
