package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
	public static class SpinPowerForTime extends Command {
		private double m_speed;
		private double m_time;
		private Timer m_timer = new Timer();

		public SpinPowerForTime(double speed, double time) {
			setName("Spine at Power for Time");
			addRequirements(Intake.getIntake());
			m_speed = speed;
			m_time = time;
		}

		@Override
		public void initialize() {
			Intake.setArmPower(m_speed);
			m_timer.reset();
		}

		@Override
		public void end(boolean interrupted) {
			Intake.setArmPower(0);
		}

		@Override
		public boolean isFinished() {
			return m_timer.hasElapsed(m_time);
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