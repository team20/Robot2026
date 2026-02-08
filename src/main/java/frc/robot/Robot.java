// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	/*
	 * private final Shooter m_shooterSubsystem = new Shooter();
	 * private final Turret m_turretSubsystem = new Turret();
	 */
	private final CommandPS5Controller m_joystick = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);

	public Robot() {
		BindDriveControls();
	}

	private void BindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.getCommand().new JoystickDrive(
						() -> -m_joystick.getLeftY(), () -> -m_joystick.getLeftX(),
						() -> m_joystick.getL2Axis() - m_joystick.getR2Axis(), m_joystick.getHID()::getCreateButton));
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
		m_scheduler.cancelAll();
		/*
		 * m_scheduler.schedule(
		 * Commands.parallel(
		 * Commands.sequence(
		 * new RunTurretToAngleHardware(m_turretSubsystem, 45),
		 * Commands.waitSeconds(1),
		 * new RunTurretToAngleHardware(m_turretSubsystem, 225),
		 * Commands.waitSeconds(1),
		 * new RunTurretToAngleHardware(m_turretSubsystem, 45),
		 * Commands.waitSeconds(1),
		 * new RunTurretToAngleHardware(m_turretSubsystem, 225)),
		 * new ShooterCommand.RunAtDynamicRPM(m_shooterSubsystem,
		 * 2400).withTimeout(40)));
		 */
		m_scheduler.schedule(
				Commands.sequence(
						m_driveSubsystem.getCommand().new DriveDistance(1, .2),
						// m_driveSubsystem.getCommand().new ResetHeading(),
						// m_driveSubsystem.getCommand().new SpinToAngle(45, .2),
						m_driveSubsystem.getCommand().new DriveDistance(-1, .2)));
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
