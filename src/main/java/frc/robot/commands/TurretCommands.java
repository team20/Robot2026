package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretCommands {

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

			Turret.getTurret().setAngle(newTurretAngle);
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
}