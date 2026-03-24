package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Aim;
import frc.robot.Constants.Subsystems.ShooterConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision.AngleDistanceEstimator;

public class AimCommands {
	/**
	 * This command uses the {@code MidpointEstimator} to estimate angle and
	 * distance to hub, and sets turret, hood, and flywheel accordingly
	 */
	public static class MidpointAim extends Command {
		private final AngleDistanceEstimator m_estimator;
		private Aim m_aim = new Aim.Linear();

		public MidpointAim(AngleDistanceEstimator estimator) {
			m_estimator = estimator;
			setName("Auto Aim Shooter and Hood");
			addRequirements(Shooter.getShooter(), Hood.getHood());
		}

		@Override
		public void execute() {
			double rotationNeeded = Turret.getAngleToTicks(m_estimator.getAngle());
			if (rotationNeeded == 0)
				return;
			double currentTurretAngle = Turret.getTurret().getPosition();
			double newTurretAngle = currentTurretAngle + rotationNeeded;

			double distance = m_estimator.getDistance().in(Feet);

			Hood.getHood().moveToPosition(m_aim.getHoodAngle(distance));
			Shooter.setRPM(m_aim.getShooterVelocity(distance));
			// Set hardware setpoint - the controller will continue to track setpoint even
			// after the command ends
			Turret.getTurret().moveToPosition(newTurretAngle);
		}

		@Override
		public boolean isFinished() {
			// This command just sets the target, controller will continue tracking after
			// commands end
			return true;
		}
	}

	public static Command getAimCommand(double distance) {
		return new SequentialCommandGroup(getSetAimCommand(distance), getSettleAimCommand());
	}

	/**
	 * Returns instantly, need to settle afterwards.
	 * 
	 * @param distance
	 * @return
	 */
	public static Command getSetAimCommand(double distance) {
		Aim aim = new Aim.Linear();
		return new SequentialCommandGroup(
				HoodCommands.getTurnToAngleCommand(aim.getHoodAngle(distance)),
				new ShooterCommands.SetRPM(aim.getShooterVelocity(distance)));
	}

	public static Command getSettleAimCommand() {
		return new SequentialCommandGroup(HoodCommands.getSettleAngleCommand(), new ShooterCommands.SettleRPM());
	}

	public static class AdjustAim extends Command {
		private static double s_distance;
		private final boolean m_absolute;
		private final double m_distance;
		private final TimedRobot m_robot;
		private final Aim m_aim = new Aim.Linear();

		/**
		 * A command to set the setpoints of the hood and shooter for aiming. In
		 * absolute mode, it sets it to the specified distance. In relative mode, it
		 * increases the distance by your specified value every second in a linear
		 * fashion.
		 * 
		 * @param absolute if the distance parameter is absolute instead of relative
		 * @param distance a specific distance in feet if in absolute or a rate in feet
		 *        per second if in relative
		 * @param robot your robot
		 */
		public AdjustAim(boolean absolute, double distance, TimedRobot robot) {
			addRequirements(Hood.getHood(), Shooter.getShooter());
			setName("Adjust aim command");
			m_absolute = absolute;
			m_distance = distance;
			m_robot = robot;
		}

		@Override
		public void execute() {
			if (m_absolute) {
				s_distance = m_distance;
			} else {
				s_distance += m_distance * m_robot.getPeriod();
			}

			SmartDashboard.putNumber("Aim Distance", s_distance);
			// ShooterState state = m_aim.getShooterState(s_distance, 0);

			Hood.getHood().moveToPosition(m_aim.getHoodAngle(s_distance));
			Shooter.setRPM(m_aim.getShooterVelocity(s_distance));
			// Clearly doable
		}

		@Override
		public boolean isFinished() {
			return m_absolute;
		}

		public static void resetAim() {
			s_distance = ShooterConstants.kDefaultDistance;
		}
	}
}
