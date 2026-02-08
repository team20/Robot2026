// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.RunTurretToAngleHardware;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final Shooter m_shooterSubsystem = new Shooter();
	private final Turret m_turretSubsystem = new Turret();
	private final CommandPS5Controller m_joystick = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);

	public Robot() {
		BindDriveControls();
	}

	private void BindDriveControls() {
		/*
		 * m_driveSubsystem.setDefaultCommand(
		 * m_driveSubsystem.driveCommand(
		 * () -> -m_joystick.getLeftY(), () -> -m_joystick.getLeftX(),
		 * () -> m_joystick.getL2Axis() - m_joystick.getR2Axis(),
		 * m_joystick.getHID()::getCreateButton));
		 * m_transportSubsystem.setDefaultCommand(
		 * m_transportSubsystem.moveWithTrigger(
		 * m_joystick.triangle(),
		 * m_joystick.cross()));
		 * m_intakeSubsystem.setDefaultCommand(
		 * m_intakeSubsystem.moveWithTrigger(
		 * m_joystick.R1(),
		 * m_joystick.L1()));
		 * m_joystick.circle().onTrue(
		 * m_intakeSubsystem.deployRollers());
		 * m_joystick.circle().onTrue(
		 * m_intakeSubsystem.retractRollers());
		 */
		/*
		 * m_shooterSubsystem.setDefaultCommand(
		 * new ShooterCommand.RunAtPower(m_shooterSubsystem, .1, 10));
		 */
		/*
		 * m_turretSubsystem.setDefaultCommand(
		 * m_turretSubsystem.aimWithJoystick(
		 * m_joystick::getLeftX,
		 * m_joystick::getLeftY));
		 * m_climberSubsystem.setDefaultCommand(
		 * m_climberSubsystem.moveWithTrigger(
		 * m_joystick.povUp(), m_joystick.povDown()));
		 */
	}

	@Override
	public void robotPeriodic() {
		m_scheduler.run();

		SmartDashboard.putData(m_scheduler);
		SmartDashboard.putNumber("rpm", m_shooterSubsystem.getRPM());
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
		m_scheduler.cancelAll();
		m_scheduler.schedule(
				Commands.parallel(
						Commands.sequence(
								new RunTurretToAngleHardware(m_turretSubsystem, 45),
								Commands.waitSeconds(1),
								new RunTurretToAngleHardware(m_turretSubsystem, 225),
								Commands.waitSeconds(1),
								new RunTurretToAngleHardware(m_turretSubsystem, 45),
								Commands.waitSeconds(1),
								new RunTurretToAngleHardware(m_turretSubsystem, 225)),
						new ShooterCommand.RunAtDynamicRPM(m_shooterSubsystem, 2400).withTimeout(40)));
	}

	@Override
	public void teleopInit() {
		m_scheduler.cancelAll();
	}

	@Override
	public void testInit() {
		m_scheduler.cancelAll();
		m_scheduler.schedule(ClampedP.testCommand());
	}
}
