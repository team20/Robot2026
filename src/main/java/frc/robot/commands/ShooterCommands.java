// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Subsystems.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
	public static class RunAtStaticRPM extends Command {
		private double m_rpm;

		/** Creates a new runShooter. */
		public RunAtStaticRPM(double rpm) {
			m_rpm = rpm;
			setName("Run Shooter At Static RPM");
			addRequirements(Shooter.getShooter());
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			Shooter.setVoltage(m_rpm / Shooter.getRPMperVolt());
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Shooter.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class RunAtPower extends Command {
		private final double m_power;
		private final double m_time;
		private final Timer m_timer = new Timer();

		/** Creates a new RunShooterAtPower. */
		public RunAtPower(double power, double time) {
			m_power = power;
			m_time = time;
			setName("Run Shooter At Power");
			addRequirements(Shooter.getShooter());
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			m_timer.start();
			Shooter.setPower(m_power);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_timer.hasElapsed(m_time);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Shooter.stop();
		}
	}

	public static class RunAtDynamicRPM extends Command {
		private double m_rpm;

		/** Creates a new runShooter. */
		public RunAtDynamicRPM(double rpm) {
			m_rpm = rpm;
			setName("Run Shooter At Dynamic RPM");
			addRequirements(Shooter.getShooter());
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			Shooter.setRPM(m_rpm);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Shooter.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class RunAtDPadRPM extends Command {
		private final Trigger m_up;
		private final Trigger m_down;
		private final IterativeRobotBase m_robot;
		private double m_rpm;

		public RunAtDPadRPM(IterativeRobotBase robot, Trigger up, Trigger down) {
			this(robot, ShooterConstants.kDefaultRPM, up, down);
		}

		public RunAtDPadRPM(IterativeRobotBase robot, double rpm, Trigger up, Trigger down) {
			m_robot = robot;
			m_rpm = rpm;
			m_up = up;
			m_down = down;
			setName("Run Shooter At D-Pad Controlled RPM");
			addRequirements(Shooter.getShooter());
		}

		@Override
		public void execute() {
			double change = 0;
			if (m_up.getAsBoolean()) {
				change += ShooterConstants.kRampRate;
			}
			if (m_down.getAsBoolean()) {
				change -= ShooterConstants.kRampRate;
			}
			m_rpm += change * m_robot.getPeriod();
			Shooter.setRPM(m_rpm);
		}

		@Override
		public void end(boolean interrupted) {
			Shooter.stop();
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}
}