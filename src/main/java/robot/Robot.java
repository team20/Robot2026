// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import robot.subsystems.Climber;
import robot.subsystems.Drive;
import robot.subsystems.Intake;
import robot.subsystems.Shooter;
import robot.subsystems.Transport;
import robot.subsystems.Turret;

/**
 * A class which ties together all of the subsystems and integrates with the
 * standard wpilib program cycle.
 */
public class Robot extends TimedRobot {
	/**
	 * Stores the command to be run in autonomous mode.
	 */
	private Command m_autonomousCommand;

	/**
	 * The drive subsystem controls the swerve modules and keeps track of odometry.
	 */
	private final Drive m_driveSubsystem = new Drive(PhysicalBot.DENNIS);
	/**
	 * The transport subsystem allows fuel to move from the intake to the shooter.
	 */
	private final Transport m_transportSubsystem = new Transport();
	/**
	 * The intake subsystem uses deployable rollers to acquire fuel and get it into
	 * our transport system.
	 */
	private final Intake m_intakeSubsystem = new Intake();
	/**
	 * The shooter subsystem allows us to launch fuel towards targets.
	 */
	private final Shooter m_shooterSubsystem = new Shooter();
	/**
	 * The turret subsystem aims the shooter at targets.
	 */
	private final Turret m_turretSubsystem = new Turret();
	/**
	 * The climber subsystem allows the robot to climb the tower.
	 */
	private final Climber m_climberSubsystem = new Climber();
	/**
	 * The sendable chooser is used to present a dropdown on elastic or the
	 * smartdashboard which contains all of our auto options.
	 */
	private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
	/**
	 * The controller allows our code to respond to inputs such as joystick movement
	 * or button presses.
	 */
	private final CommandPS5Controller m_joystick = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);

	/**
	 * Runs all of the robot code.
	 * 
	 * @param args program arguments
	 */
	public static void main(String... args) {
		RobotBase.startRobot(Robot::new);
	}

	/**
	 * Initializes our robot by binding the teleop controls.
	 */
	public Robot() {
		bindTeleopControls();
	}

	/**
	 * Assigns default commands to each subsystem which respond to controller inputs
	 * during teleop.
	 */
	private void bindTeleopControls() {
		/**
		 * Bind the drive subsystem to joysticks for translation and triggers for
		 * rotation.
		 */
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_joystick.getLeftY(), () -> -m_joystick.getLeftX(),
						() -> m_joystick.getL2Axis() - m_joystick.getR2Axis(), m_joystick.getHID()::getCreateButton));
		/**
		 * Bind the transport subsystem to triangle for forwards and cross for
		 * backwards.
		 */
		m_transportSubsystem.setDefaultCommand(
				m_transportSubsystem.moveWithTrigger(
						m_joystick.triangle(),
						m_joystick.cross()));
		/**
		 * Bind the intake subsystem to right bumper for intake and the left bumper for
		 * eject.
		 */
		m_intakeSubsystem.setDefaultCommand(
				m_intakeSubsystem.moveWithTrigger(
						m_joystick.R1(),
						m_joystick.L1()));
		/**
		 * Bind the intake subsystem to deploy rollers when circle is pressed.
		 */
		m_joystick.circle().onTrue(
				m_intakeSubsystem.deployRollers());
		/**
		 * Bind the intake subsystem to retract rollers when circle is released.
		 */
		m_joystick.circle().onFalse(
				m_intakeSubsystem.retractRollers());
		/**
		 * Bind the shooter subsystem to spin the shooter when square is pressed.
		 */
		m_shooterSubsystem.setDefaultCommand(
				m_shooterSubsystem.moveWithTrigger(
						m_joystick.square(), null));
		/**
		 * Bind the turret subsystem to aim the shooter using the left joystick.
		 */
		m_turretSubsystem.setDefaultCommand(
				m_turretSubsystem.aimWithJoystick(
						m_joystick::getLeftX,
						m_joystick::getLeftY));
		/**
		 * Bind the climber system to use the D-pad for both up and down.
		 */
		m_climberSubsystem.setDefaultCommand(
				m_climberSubsystem.moveWithTrigger(
						m_joystick.povUp(), m_joystick.povDown()));
	}

	/**
	 * Runs the command scheduler on each iteration of the periodic.
	 */
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	/**
	 * Begin to run the autonomous command when autonomous starts.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_autoChooser.getSelected();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/**
	 * Stop running the autonomous command when teleop starts.
	 */
	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}
}
