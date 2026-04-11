package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Aim;
import frc.robot.Constants.Subsystems.ShooterConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision.AngleDistanceEstimator;

public class AimCommands {
	private static double s_airtime = 1.2; // Set default airtime to 1.2

	// Return updated airtime based on changing distance
	public static double getAirtime() {
		return s_airtime;
	}

	// This command uses the {@code AngleDistanceEstimator} - either field
	// pose or midpoint to estimate angle and distance to hub, and sets
	// turret, hood, and flywheel accordingly
	public static class HubAimCommand extends Command {
		private final AngleDistanceEstimator m_estimator;

		public HubAimCommand(AngleDistanceEstimator estimator) {
			m_estimator = estimator;
			setName("Auto Aim Shooter, Hood, and Turret");
			addRequirements(Shooter.getShooter(), Hood.getHood(), Turret.getTurret());
		}

		@Override
		public void execute() {

			{ // Hood and shooter adjustment
				double distance = m_estimator.getDistance().in(Feet);
				SmartDashboard.putNumber("Vision/AIM DISTANCE", distance);
				s_airtime = Aim.getShotAirtime(distance); // Update airtime based on new distance
				Hood.getHood().moveToPosition(Aim.getHoodAngle(distance));
				Shooter.setRPM(Aim.getShooterVelocity(distance));
			}

			{ // Turret rotation

				// double rotationNeeded = Turret.getAngleToTicks(m_estimator.getAngle());
				// if (rotationNeeded == 0)
				// return;
				// double currentTurretAngle = Turret.getTurret().getPosition();
				// double newTurretAngle = currentTurretAngle + rotationNeeded;

				// Get mixed robot pose (odometry + vision) and compensated hub pose
				Pose2d robotPose = Drive.getPose();
				Pose2d target = Drive.getAimTarget();

				// Find x and y components of vector from bot to hub
				double dx = target.getX() - robotPose.getX();
				double dy = -target.getY() + robotPose.getY(); // Inverted in coordinate space

				// Calculate vector angle (in world-space) and convert to turret position
				double hubAngle = Math.toDegrees(Math.atan2(dy, dx));
				double hubTicks = Turret.getTurret().getTurretToWorldAngleRotation(robotPose, hubAngle);

				// Set hardware setpoint - the controller will continue to
				// track setpoint even after the command ends
				Turret.getTurret().moveToPosition(hubTicks);
			}
		}

		@Override
		public boolean isFinished() {
			// This command just sets the target, controller will
			// continue tracking after commands end
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
		return new SequentialCommandGroup(
				HoodCommands.getTurnToAngleCommand(Aim.getHoodAngle(distance)),
				new ShooterCommands.SetRPM(Aim.getShooterVelocity(distance)));
	}

	public static Command getSettleAimCommand() {
		return new SequentialCommandGroup(HoodCommands.getSettleAngleCommand(), new ShooterCommands.SettleRPM());
	}

	public static class AdjustAim extends Command {
		private static double s_distance;
		private final boolean m_absolute;
		private final double m_distance;
		private final TimedRobot m_robot;

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

			Hood.getHood().moveToPosition(Aim.getHoodAngle(s_distance));
			Shooter.setRPM(Aim.getShooterVelocity(s_distance));
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
