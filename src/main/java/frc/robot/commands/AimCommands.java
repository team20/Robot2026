package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
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

		public HubAimCommand() {
			setName("Auto Aim Shooter, Hood, and Turret");
			addRequirements(Shooter.getShooter(), Hood.getHood(), Turret.getTurret());
		}

		@Override
		public void execute() {

			// Get mixed robot pose (odometry + vision) and compensated hub pose
			Pose2d robotPose = Drive.getPose();
			Pose2d target = Drive.getVelocityCompensatedAimTarget();

			// Find x and y components of vector from bot to hub
			double dx = target.getX() - robotPose.getX();
			double dy = -target.getY() + robotPose.getY(); // Inverted in coordinate space
			Distance distance = Meters.of(Math.sqrt(dx * dx + dy * dy));

			{ // Turret rotation
				// Calculate vector angle (in world-space) and convert to turret position
				double hubAngle = Math.toDegrees(Math.atan2(dy, dx));
				double hubTicks = Turret.getTurret().getTurretToWorldAngleRotation(robotPose, hubAngle);

				// Set hardware setpoint - the controller will continue to
				// track setpoint even after the command ends
				Turret.getTurret().moveToPosition(hubTicks);
			}

			{ // Hood and shooter adjustment
				s_airtime = Aim.getShotAirtime(distance); // Update airtime based on new distance
				Hood.getHood().moveToPosition(Aim.getHoodAngle(distance));
				Shooter.setRPM(Aim.getShooterVelocity(distance));
			}

		}

		@Override
		public boolean isFinished() {
			// This command just sets the target, controller will
			// continue tracking after commands end
			return true;
		}
	}

	public static Command getAimCommand(Distance distance) {
		return new SequentialCommandGroup(getSetAimCommand(distance), getSettleAimCommand());
	}

	/**
	 * Returns instantly, need to settle afterwards.
	 * 
	 * @param distance
	 * @return
	 */
	public static Command getSetAimCommand(Distance distance) {
		return new SequentialCommandGroup(
				HoodCommands.getTurnToAngleCommand(Aim.getHoodAngle(distance)),
				new ShooterCommands.SetRPM(Aim.getShooterVelocity(distance)));
	}

	public static Command getSettleAimCommand() {
		return new SequentialCommandGroup(HoodCommands.getSettleAngleCommand(), new ShooterCommands.SettleRPM());
	}

	public static class AdjustAim extends Command {
		private static Distance s_distance;
		private final boolean m_absolute;
		private final Distance m_distance;
		private final TimedRobot m_robot;

		/**
		 * A command to set the setpoints of the hood and shooter for aiming. In
		 * absolute mode, it sets it to the specified distance. In relative mode, it
		 * increases the distance by your specified value every second in a linear
		 * fashion.
		 * 
		 * @param absolute if the distance parameter is absolute instead of relative
		 * @param distance a specific distance if in absolute or a rate per second if in
		 *        relative
		 * @param robot your robot
		 */
		public AdjustAim(boolean absolute, Distance distance, TimedRobot robot) {
			addRequirements(Hood.getHood(), Shooter.getShooter());
			setName("Adjust aim command");
			m_absolute = absolute;
			m_distance = distance;
			m_robot = robot;
		}

		@Override
		public void execute() {
			if (m_absolute) {
				// Set distance
				s_distance = m_distance;
			} else {
				// Apply rate
				s_distance = s_distance.plus(m_distance.times(m_robot.getPeriod()));
			}

			SmartDashboard.putNumber("Aim Distance Feet", s_distance.in(Feet));
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
