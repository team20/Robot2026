package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretCommands {

	public static Command getTurnToAngleCommand(double angle) {
		return AngularPositionCommands.RunToAngleHardware(
				Turret.getTurret(), angle,
				Turret.getConstants().tolerance());
	}

	public static Command getSetAngleCommand(double angle) {
		return new AngularPositionCommands.SetAngleHardware(Turret.getTurret(), angle);
	}

	public static Command getSettleAngleCommand() {
		return new AngularPositionCommands.SettleAngle(Turret.getTurret(), Turret.getConstants().tolerance());
	}

	public static Command getTurnToAngleSoftwareCommand(double angle) {
		return new AngularPositionCommands.RunToAngleSoftware(Turret.getTurret(), angle, Turret.getConstants());
	}

	public static class GradualAim extends Command {
		private Turret m_turret;
		private double m_maxSpeed, m_minSpeed;
		private double m_deadzone;
		private DoubleSupplier m_left, m_right;

		public GradualAim(double minSpeed, double maxSpeed, double deadzone, DoubleSupplier left,
				DoubleSupplier right) {
			m_turret = Turret.getTurret();
			m_maxSpeed = maxSpeed;
			m_minSpeed = minSpeed;
			m_deadzone = deadzone;
			m_left = left;
			m_right = right;

			addRequirements(m_turret);
			setName("Gradual turret aim");
		}

		@Override
		public void execute() {
			double left = Math.pow((m_left.getAsDouble() + 1) / 2, 2);
			double right = Math.pow((m_right.getAsDouble() + 1) / 2, 2);
			if (Math.abs(right - left) > m_deadzone) {
				double value = (m_minSpeed * Math.signum(right - left)) + (right - left) * (m_maxSpeed - m_minSpeed);
				m_turret.runAtDutyCycle(value);
			} else {
				m_turret.stop();
			}
		}

		@Override
		public void end(boolean interrupted) {
			m_turret.stop();
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}
}