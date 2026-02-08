// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand {
	/*
	 * You should consider using the more terse Command factories API instead
	 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-
	 * command-based.html#defining-commands
	 */
	public static class RunAtStaticRPM extends Command {

		private Shooter m_shooter;
		private double m_rpm;

		/** Creates a new runShooter. */
		public RunAtStaticRPM(Shooter shooter, double rpm) {
			m_shooter = shooter;
			m_rpm = rpm;
			setName("Run Shooter At RPM");
			addRequirements(shooter);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			double kv = m_shooter.getRPMperVolt();
			double voltage = m_rpm / kv;
			m_shooter.setVoltage(voltage);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_shooter.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class RunAtPower extends Command {

		private final Shooter m_shooter;
		private final double m_power;
		private final double m_time;
		private final Timer m_timer = new Timer();

		/** Creates a new RunShooterAtPower. */
		public RunAtPower(Shooter shooter, double power, double time) {
			m_shooter = shooter;
			m_power = power;
			m_time = time;
			setName("Run Shooter At Power");
			addRequirements(shooter);
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			m_timer.start();
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			m_shooter.setPower(m_power);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_timer.hasElapsed(m_time);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_shooter.stop();
		}
	}

	public static class RunAtDynamicRPM extends Command {

		private Shooter m_shooter;
		private VelocityVoltage m_request;

		/** Creates a new runShooter. */
		public RunAtDynamicRPM(Shooter shooter, double rpm) {
			m_shooter = shooter;
			m_request = new VelocityVoltage(RPM.of(rpm))
					.withFeedForward(rpm / m_shooter.getRPMperVolt());

			setName("Run Shooter At RPM");
			addRequirements(shooter);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			m_shooter.velocityVoltage(m_request);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_shooter.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}
}