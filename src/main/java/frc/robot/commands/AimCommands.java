package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Aim;
import frc.robot.Constants.Subsystems.ShooterConstants;
import frc.robot.subsystems.AngularPositionSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AimCommands {

	// This command uses the {@code MidpointEstimator} to estimate angle and
	// distance to hub, and sets turret, hood, and flywheel accordingly
	public static class MidpointAim extends Command {
		private final AngularPositionSubsystem m_turret;
		private final Vision m_vision;
		private double m_distance;
		private Aim m_aim = new Aim.Linear();

		public MidpointAim(AngularPositionSubsystem turret, Vision vision) {
			m_turret = turret;
			m_vision = vision;
			setName("Auto Aim Shooter and Hood");
			addRequirements(Shooter.getShooter(), Hood.getHood(), turret, vision);
		}

		@Override
		public void execute() {
			double rotationNeeded = m_vision.getAngleToHubTag();
			if (rotationNeeded == 0)
				return;
			double currentTurretAngle = Turret.getTurret().getPosition();
			double newTurretAngle = currentTurretAngle + rotationNeeded;

			m_distance = m_vision.getDistanceToHub();

			Hood.getHood().moveToPosition(m_aim.getHoodAngle(m_distance));
			Shooter.setRPM(m_aim.getShooterVelocity(m_distance));
			// Set hardware setpoint - the controller will continue to track setpoint even
			// after the command ends
			Turret.getTurret().moveToPosition(newTurretAngle);

			SmartDashboard.putNumber("Aim Distance", m_distance);
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

	public static class RiyaAiming extends Command {
		private double m_distance;
		private Aim m_aim = new Aim.Linear();

		public RiyaAiming(Trigger farther, Trigger closer, Map<Trigger, Double> bindings) {
			this(ShooterConstants.kDefaultDistance, farther, closer, bindings);
		}

		public RiyaAiming(double distance, Trigger farther, Trigger closer,
				Map<Trigger, Double> bindings) {
			for (Map.Entry<Trigger, Double> binding : bindings.entrySet()) {
				double range = binding.getValue();
				binding.getKey().onTrue(Commands.runOnce(() -> {
					m_distance = range;
				}));
			}
			farther.onTrue(Commands.runOnce(() -> {
				m_distance += ShooterConstants.kRampRate * .1;
			}));
			closer.onTrue(Commands.runOnce(() -> {
				m_distance -= ShooterConstants.kRampRate * .1;
			}));
			m_distance = distance;
			setName("Aim Shooter and Hood Using D-Pad");
			addRequirements(Shooter.getShooter(), Hood.getHood());
		}

		@Override
		public void execute() {
			Hood.getHood().moveToPosition(m_aim.getHoodAngle(m_distance));
			Shooter.setRPM(m_aim.getShooterVelocity(m_distance));
		}

		@Override
		public void end(boolean interrupted) {
			Shooter.stop();
			Hood.getHood().stop();
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class AimWDPad extends Command {
		private final Trigger m_farther;
		private final Trigger m_closer;
		private final IterativeRobotBase m_robot;
		private final Trigger[] m_bindings;
		private final double[] m_distances;
		private double m_distance;
		private Aim m_aim = new Aim.Linear();

		public AimWDPad(IterativeRobotBase robot, Trigger farther, Trigger closer, Map<Trigger, Double> bindings) {
			this(robot, ShooterConstants.kDefaultDistance, farther, closer, bindings);
		}

		public AimWDPad(IterativeRobotBase robot, double distance, Trigger farther, Trigger closer,
				Map<Trigger, Double> bindings) {
			m_bindings = new Trigger[bindings.size()];
			m_distances = new double[bindings.size()];
			int i = 0;
			for (Map.Entry<Trigger, Double> binding : bindings.entrySet()) {
				m_bindings[i] = binding.getKey();
				m_distances[i] = binding.getValue();
				i++;
			}
			m_robot = robot;
			m_farther = farther;
			m_closer = closer;
			m_distance = distance;
			setName("Aim Shooter and Hood Using D-Pad");
			addRequirements(Shooter.getShooter(), Hood.getHood());
		}

		@Override
		public void execute() {
			double change = 0;
			if (m_farther.getAsBoolean()) {
				change += ShooterConstants.kRampRate;
			}
			if (m_closer.getAsBoolean()) {
				change -= ShooterConstants.kRampRate;
			}
			m_distance += change * m_robot.getPeriod();
			for (int i = 0; i < m_bindings.length; i++) {
				if (m_bindings[i].getAsBoolean()) {
					m_distance = m_distances[i];
				}
			}
			Hood.getHood().moveToPosition(m_aim.getHoodAngle(m_distance));
			Shooter.setRPM(m_aim.getShooterVelocity(m_distance));
		}

		@Override
		public void end(boolean interrupted) {
			Shooter.stop();
			Hood.getHood().stop();
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}
}
