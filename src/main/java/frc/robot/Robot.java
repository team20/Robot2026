// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();
	SendableChooser<Boolean> m_botChooser = new SendableChooser<>();
	public static boolean isCompBot;

	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	/*
	 * private final Shooter m_shooterSubsystem = new Shooter();
	 * private final Turret m_turretSubsystem = new Turret();
	 */
	private final CommandPS5Controller m_joystick = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);

	public Robot() {
		SmartDashboard.putData("Robot Chooser", m_botChooser);
		addBotOptions();
	}

	private void addBotOptions() {
		m_botChooser.addOption("Comp Bot", true);
		m_botChooser.addOption("Practice Bot", false);
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
		isCompBot = m_botChooser.getSelected(); // TODO: should this stay here?
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
				new SequentialCommandGroup(
						m_driveSubsystem.getCommand().new DriveDistance(1.5)));

		/*
		 * m_scheduler.schedule(
		 * new SequentialCommandGroup(
		 * m_driveSubsystem.getCommand().new DriveDistanceForTime(10, 2),
		 * m_driveSubsystem.getCommand().new TurnSteerToAngle(45),
		 * m_driveSubsystem.getCommand().new DriveDistanceForTime(10, 2),
		 * m_driveSubsystem.getCommand().new TurnSteerToAngle(90),
		 * m_driveSubsystem.getCommand().new DriveDistanceForTime(10, 2),
		 * m_driveSubsystem.getCommand().new TurnSteerToAngle(135)));
		 */
	}

	@Override
	public void teleopInit() {
		// System.out.println("Ah-ha!");
		m_scheduler.cancelAll();
		BindDriveControls();
	}

	@Override
	public void testInit() {
		m_scheduler.cancelAll();
		m_scheduler.schedule(ClampedP.testCommand());
	}
}
