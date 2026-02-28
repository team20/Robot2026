package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Kicker;

public class TransportCommands {
	public static class RunKickerAtPower extends Command {
		private final double m_power;
		private final double m_time;
		private final Timer m_timer = new Timer();

		/** Creates a new RunShooterAtPower. */
		public RunKickerAtPower(double power, double time) {
			m_power = power;
			m_time = time;
			setName("Run kicker At Power");
			addRequirements(Kicker.getKicker());
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			m_timer.start();
			Kicker.setPower(m_power);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_time > 0 && m_timer.hasElapsed(m_time);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Kicker.stop();
		}
	}

	public static class StopKicker extends Command {

		/** Creates a new StopKicker. */
		public StopKicker() {
			setName("Stop Kicker");
			addRequirements(Kicker.getKicker());
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			Kicker.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class RunAgitatorAtPower extends Command {
		private final double m_power;
		private final double m_time;
		private final Timer m_timer = new Timer();

		/** Creates a new RunShooterAtPower. */
		public RunAgitatorAtPower(double power, double time) {
			m_power = power;
			m_time = time;
			setName("Run agitator At Power");
			addRequirements(Agitator.getAgitator());
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			m_timer.start();
			Agitator.setPower(m_power);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_time > 0 && m_timer.hasElapsed(m_time);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Agitator.stop();
		}
	}

	public static class StopAgitator extends Command {

		/** Creates a new StopKicker. */
		public StopAgitator() {
			setName("Stop Agitator");
			addRequirements(Agitator.getAgitator());
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			Agitator.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}
}