package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedP;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
	public static class SpinArmPowerForTime extends Command {
		private double m_speed;
		private double m_time;
		private Timer m_timer = new Timer();

		public SpinArmPowerForTime(double speed, double time) {
			setName("Spine at Power for Time");
			addRequirements(Intake.getIntake());
			m_speed = speed;
			m_time = time;
		}

		@Override
		public void initialize() {
			Intake.setArmPower(m_speed);
			m_timer.reset();
			m_timer.start();
		}

		@Override
		public void end(boolean interrupted) {
			Intake.stopArm();
		}

		@Override
		public boolean isFinished() {
			return m_timer.hasElapsed(m_time);
		}
	}

	public static class SpinArmPower extends Command {
		private double m_speed;
		private double m_time;

		public SpinArmPower(double speed) {
			setName("Spine at Power for Time");
			addRequirements(Intake.getIntake());
			m_speed = speed;
		}

		@Override
		public void initialize() {
			Intake.setArmPower(m_speed);
		}

		@Override
		public void end(boolean interrupted) {
			Intake.stopArm();
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class MoveArmToPosition extends Command {
		double m_position;

		public MoveArmToPosition(double position) {
			m_position = position;
		}

		@Override
		public void execute() {
			Intake.setArmPower(ClampedP.clampedP(Intake.getArmRotations() - m_position, 0.1, 1, 0.5, 0.01));
		}

		@Override
		public void end(boolean interrupted) {
			Intake.stopArm();
		}

		@Override
		public boolean isFinished() {
			return Math.abs(m_position - Intake.getArmRotations()) <= 0.01;
		}
	}

	public static class ExtendArmCommand extends Command {
		public ExtendArmCommand() {
			setName("Extend Intake Arm");
			addRequirements(Intake.getIntake());
		}

		public void initialize() {
			Intake.setArmPower(IntakeConstants.kArmPower);
		}

		// Update this to use limit switch instead of getArmAngle method.
		public boolean isFinished() {
			return false;
		}

		public void end() {
			Intake.stopArm();
		}
	}

	public static class RetractArmCommand extends Command {
		public RetractArmCommand() {
			setName("Retract Intake Arm");
			addRequirements(Intake.getIntake());
		}

		public void initialize() {
			Intake.setArmPower(-IntakeConstants.kArmPower);
		}

		// Update this to use limit switch instead of getArmAngle method.
		public boolean isFinished() {
			return Intake.isReverseLimitActive();
		}

		public void end() {
			Intake.stopArm();
		}
	}

	public static class Spin extends Command {
		private final double m_speed;

		public Spin(double speed) {
			m_speed = speed;
			setName("Spin Intake");
		}

		public void initialize() {
			Intake.setWheelPower(m_speed);
		}

		// Update this to use limit switch instead of getArmAngle method.
		public boolean isFinished() {
			return false;
		}

		public void end() {
			Intake.stopWheel();
		}
	}
}