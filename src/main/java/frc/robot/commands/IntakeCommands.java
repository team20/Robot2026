package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheels;

public class IntakeCommands {
	public static class SpinIntake extends Command {
		private double m_speed;

		public SpinIntake(double speed) {
			m_speed = speed;
		}

		@Override
		public void initialize() {
			IntakeWheels.setWheelPower(m_speed);
		}

		public void end() {
			IntakeWheels.stopWheel();
		}
	}

	public static class StopIntake extends Command {
		@Override
		public void initialize() {
			IntakeWheels.stopWheel();
		}

		@Override
		public boolean isFinished() {
			return true;
		}
	}
}