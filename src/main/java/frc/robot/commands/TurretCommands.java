package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

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

	public static class TurretAimAtTag extends Command {
		private final Vision m_vision;

		public TurretAimAtTag(Vision vision) {
			m_vision = vision;
			setName("Turret Aim At Tag");

			addRequirements(Turret.getTurret(), vision);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void initialize() {
			double rotationNeeded = m_vision.getAngleToHubTag();
			if (rotationNeeded == 0)
				return;

			double currentTurretAngle = Turret.getTurret().getPosition();
			double newTurretAngle = currentTurretAngle + rotationNeeded;

			Turret.getTurret().moveToAngle(newTurretAngle);
		}

		// Called once the command ends or is interrupted.
		// @Override
		// public void end(boolean interrupted) {
		// Turret.getTurret().stop();
		// }
		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class GradualAim extends Command {
		private Turret m_turret;
		private double m_maxSpeed;
		private double m_deadzone;
		private DoubleSupplier m_left;
		private DoubleSupplier m_right;

		public GradualAim(double maxSpeed, double deadzone, DoubleSupplier left, DoubleSupplier right) {
			m_turret = Turret.getTurret();
			m_maxSpeed = maxSpeed;
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
			double value = MathUtil.applyDeadband(right - left, m_deadzone) * m_maxSpeed;
			m_turret.runAtDutyCycle(value);
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