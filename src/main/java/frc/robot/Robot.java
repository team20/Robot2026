// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.RunTurretToAngleHardware;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	// private final Drive m_driveSubsystem = new Drive();
	private final Shooter m_shooterSubsystem = new Shooter();
	private final Turret m_turretSubsystem = new Turret();
	private final CommandPS4Controller m_driverController = new CommandPS4Controller(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandPS4Controller m_operatorController = new CommandPS4Controller(
			Constants.ControllerConstants.kOperatorControllerPort);

	public Robot() {
		BindDriveControls();
	}

	private void BindDriveControls() {
		m_operatorController.triangle()
				.toggleOnTrue(new ShooterCommands.Toggle(m_shooterSubsystem));

		m_operatorController.povDown().and(() -> m_shooterSubsystem.getRPM() != 0).onTrue(
				runOnce(
						() -> m_shooterSubsystem.setRPMSetpoint(m_shooterSubsystem.getRPMSetpoint() - 10),
						m_shooterSubsystem));

		m_operatorController.povUp().and(() -> m_shooterSubsystem.getRPM() != 0).onTrue(
				runOnce(
						() -> m_shooterSubsystem.setRPMSetpoint(m_shooterSubsystem.getRPMSetpoint() + 10),
						m_shooterSubsystem));
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
						new ShooterCommands.RunAtDynamicRPM(m_shooterSubsystem, 2400).withTimeout(40)));
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
