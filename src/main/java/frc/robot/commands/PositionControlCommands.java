package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedP;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.PositionControlSubsystem;

public class PositionControlCommands {
	public static class SpinMotorPowerForTime extends Command {
		private double m_speed;
		private double m_time;
		private PositionControlSubsystem m_subsystem;
		private Timer m_timer = new Timer();

		public SpinMotorPowerForTime(PositionControlSubsystem subsystem, double speed, double time) {
			setName("Spin at Power for Time");
			addRequirements(subsystem);
			m_speed = speed;
			m_time = time;
			m_subsystem = subsystem;
		}

		@Override
		public void initialize() {
			m_subsystem.setMotorPower(m_speed);
			m_timer.reset();
			m_timer.start();
		}

		@Override
		public void end(boolean interrupted) {
			m_subsystem.stopMotor();
		}

		@Override
		public boolean isFinished() {
			return m_timer.hasElapsed(m_time);
		}
	}

	public static class SpinMotorPower extends Command {
		private double m_speed;
		private PositionControlSubsystem m_subsystem;

		public SpinMotorPower(PositionControlSubsystem subsystem, double speed) {
			setName("Spin at Power for Time");
			addRequirements(subsystem);
			m_speed = speed;
			m_subsystem = subsystem;
		}

		@Override
		public void initialize() {
			m_subsystem.setMotorPower(m_speed);
		}

		@Override
		public void end(boolean interrupted) {
			m_subsystem.stopMotor();
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class MoveMotorToPosition extends Command {
		private double m_position;
		private PositionControlSubsystem m_subsystem;
		private boolean m_hold;
		private double m_minPower;
		private double m_maxPower;
		private double m_maxError;
		private double m_tolerance;

		// position is an absolute position, in number of rotations,
		// relative to the last time the subsytem was zeroed
		public MoveMotorToPosition(PositionControlSubsystem subsystem, double position, double minPower,
				double maxPower, double maxError, double tolerance, boolean hold) {
			addRequirements(subsystem);
			m_position = position;
			m_subsystem = subsystem;
			m_minPower = minPower;
			m_maxPower = maxPower;
			m_maxError = maxError;
			m_tolerance = tolerance;
		}

		@Override
		public void execute() {
			double error = m_subsystem.getMotorRotations() - m_position;
			m_subsystem.setMotorPower(ClampedP.clampedP(error, m_minPower, m_maxPower, m_maxError, m_tolerance));
		}

		@Override
		public void end(boolean interrupted) {
			m_subsystem.stopMotor();
		}

		@Override
		public boolean isFinished() {
			if (m_hold) {
				return false;
			}

			return Math.abs(m_position - m_subsystem.getMotorRotations()) <= m_tolerance;
		}
	}

	public static class SpinIntake extends Command {
		private final double m_speed;

		public SpinIntake(double speed) {
			m_speed = speed;
			setName("Spin Intake");
		}

		public void initialize() {
			IntakeWheels.setWheelPower(m_speed);
		}

		// Update this to use limit switch instead of getArmAngle method.
		public boolean isFinished() {
			return false;
		}

		public void end() {
			IntakeWheels.stopWheel();
		}
	}

	public static class ResetEncoder extends Command {
		@Override
		public void initialize() {
			IntakeArm.getIntakeArm().resetMotorEncoder();
		}
	}
}
