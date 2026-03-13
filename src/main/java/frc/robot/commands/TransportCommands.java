package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Subsystems.AgitatorConstants;
import frc.robot.subsystems.Agitator;

public class TransportCommands {
	public static Command getTimedShoot(double time) {
		return Commands.parallel(
				new RunAgitatorAtPowerAndTime(AgitatorConstants.kTeleopPower, time));
	}

	public static class RunAgitatorAtPowerAndTime extends Command {
		private final double m_power;
		private final double m_time;
		private final Timer m_timer = new Timer();

		/** Creates a new RunShooterAtPower. */
		public RunAgitatorAtPowerAndTime(double power, double time) {
			m_power = power;
			m_time = time;
			setName("Run agitator At Power and Time");
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

	public static class RunAgitatorAtPower extends Command {
		private final double m_power;

		/** Creates a new RunShooterAtPower. */
		public RunAgitatorAtPower(double power) {
			m_power = power;
			setName("Run agitator At Power");
			addRequirements(Agitator.getAgitator());
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			Agitator.setPower(m_power);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
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