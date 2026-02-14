
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transport;

public class TransportCommands {
	public static class RunAtPower extends Command {
		private final double m_power;
		private final double m_time;
		private final Timer m_timer = new Timer();

		/** Creates a new RunShooterAtPower. */
		public RunAtPower(double power, double time) {
			m_power = power;
			m_time = time;
			setName("Run transport At Power");
			addRequirements(Transport.getTransport());
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			m_timer.start();
			Transport.setPower(m_power);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_timer.hasElapsed(m_time);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Transport.stop();
		}
	}
}