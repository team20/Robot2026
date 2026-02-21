package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.ShooterCommands;

public abstract class Aim {
	public record ShooterState(double turretAngle, double hoodAngle, double shooterVelocity) {
	}

	protected abstract double getShooterVelocity(double distance);

	protected abstract double getHoodAngle(double distance);

	public ShooterState getShooterState(double x, double y) {
		double distance = Math.hypot(x, y);
		double turretAngle = Math.atan2(y, x);
		return new ShooterState(turretAngle, getHoodAngle(distance), getShooterVelocity(distance));
	}

	public Command getAimCommand(double distance) {
		ShooterState state = getShooterState(distance, 0);
		return Commands.parallel(
				new ShooterCommands.RunAtDynamicRPM(state.shooterVelocity),
				new HoodCommands.RunToAngleHardware(state.hoodAngle));
	}

	public static class Regression extends Aim {
		private static final double s_velocityA = 0;
		private static final double s_velocityB = 0;
		private static final double s_velocityC = 0;
		private static final double s_angleA = 0;
		private static final double s_angleB = 0;
		private static final double s_angleC = 0;

		@Override
		protected double getShooterVelocity(double distance) {
			return (s_velocityA * distance + s_velocityB) * distance + s_velocityC;
		}

		@Override
		protected double getHoodAngle(double distance) {
			return (s_angleA * distance + s_angleB) * distance + s_angleC;
		}

	};

	public static class Linear extends Aim {
		private static final double[] s_distances = new double[] { 4, 8, 12, 15 };
		private static final double[] s_angles = new double[] { 8.5, 24, 30, 38 };
		private static final double[] s_velocities = new double[] { 2050, 2200, 2550, 2750 };

		private static double interpolate(double a, double b, double t) {
			return (1 - t) * a + t * b;
		}

		private static int getIndex(double distance) {
			for (int i = 0; i < s_distances.length - 1; i++) {
				if (s_distances[i] <= distance && s_distances[i + 1] > distance) {
					return i;
				}
			}
			return -1;
		}

		private static double interpolate(double distance, double[] list) {
			int index = getIndex(distance);
			if (index < 0) {
				return -1;
			} else {
				double a = list[index];
				double b = list[index + 1];
				double delta = s_distances[index + 1] - s_distances[index];
				double t = (distance - s_distances[index]) / delta;
				return interpolate(a, b, t);
			}
		}

		public static Command testCommand() {
			return Commands.runOnce(() -> {
				Aim aimer = new Interpolation();
				for (int i = 0; i < s_distances.length - 1; i++) {
					double middle = (s_distances[i] + s_distances[i + 1]) / 2;
					double velocity = aimer.getShooterVelocity(middle);
					double angle = aimer.getHoodAngle(middle);
					double maxV = Math.max(s_velocities[i], s_velocities[i + 1]);
					double maxErr = 0.01;
					if (velocity > maxV && (velocity - maxV) / maxV > maxErr) {
						throw new Error(String.format("Velocity interpolation was too high (%f > %f)", velocity, maxV));
					}
					double minV = Math.min(s_velocities[i], s_velocities[i + 1]);
					if (velocity < minV && (minV - velocity) / minV > maxErr) {
						throw new Error(String.format("Velocity interpolation was too low (%f < %f)", velocity, minV));
					}
					double maxA = Math.max(s_angles[i], s_angles[i + 1]);
					if (angle > maxA && (angle - maxA) / maxA > maxErr) {
						throw new Error(String.format("Angle interpolation was too high (%f > %f)", angle, maxA));
					}
					double minA = Math.min(s_angles[i], s_angles[i + 1]);
					if (angle < minA && (minA - angle) / minA > maxErr) {
						throw new Error(String.format("Angle interpolation was too low (%f < %f)", angle, minA));
					}
				}
				System.out.println("All interpolation tests passing!!!!!!!");
			});
		}

		@Override
		protected double getShooterVelocity(double distance) {
			return interpolate(distance, s_velocities);
		}

		@Override
		protected double getHoodAngle(double distance) {
			return interpolate(distance, s_angles);
		}
	}

	public static class Interpolation extends Aim {
		private static final double[] s_distances = new double[] { 4, 8, 12, 15 };
		private static final double[] s_angles = new double[] { 8.5, 24, 30, 38 };
		private static final double[] s_velocities = new double[] { 2050, 2200, 2550, 2750 };

		private static double interpolate(double a, double b, double da, double db, double t) {
			double result = 0;
			result += ((2 * t - 3) * t * t + 1) * a;
			result += ((t - 2) * t + 1) * t * da;
			result += (3 - 2 * t) * t * t * b;
			result += (t - 1) * t * t * db;
			return result;
		}

		private static int getIndex(double distance) {
			for (int i = 0; i < s_distances.length - 1; i++) {
				if (s_distances[i] <= distance && s_distances[i + 1] > distance) {
					return i;
				}
			}
			return -1;
		}

		private static double interpolate(double distance, double[] list) {
			int index = getIndex(distance);
			if (index < 0) {
				return -1;
			} else {
				int end = s_distances.length - 2;
				double a = list[index];
				double b = list[index + 1];
				double da;
				double db;
				if (index == 0) {
					da = (list[1] - list[0]) / (s_distances[1] - s_distances[0]);
					db = ((list[2] - list[1]) / (s_distances[2] - s_distances[1]) + da) / 2;
				} else if (index == end) {
					db = (list[end + 1] - list[end]) / (s_distances[end + 1] - s_distances[end]);
					da = ((list[end] - list[end - 1]) / (s_distances[end] - s_distances[end - 1]) + db) / 2;
				} else {
					double d = (b - a) / (s_distances[index + 1] - s_distances[index]);
					da = ((a - list[index - 1]) / (s_distances[index] - s_distances[index - 1]) + d) / 2;
					db = ((list[index + 2] - b) / (s_distances[index + 2] - s_distances[index + 1]) + d) / 2;
				}
				double delta = s_distances[index + 1] - s_distances[index];
				da *= delta;
				db *= delta;
				double t = (distance - s_distances[index]) / delta;
				return interpolate(a, b, da, db, t);
			}
		}

		public static Command testCommand() {
			return Commands.runOnce(() -> {
				Aim aimer = new Interpolation();
				for (int i = 0; i < s_distances.length - 1; i++) {
					double middle = (s_distances[i] + s_distances[i + 1]) / 2;
					double velocity = aimer.getShooterVelocity(middle);
					double angle = aimer.getHoodAngle(middle);
					double maxV = Math.max(s_velocities[i], s_velocities[i + 1]);
					double maxErr = 0.01;
					if (velocity > maxV && (velocity - maxV) / maxV > maxErr) {
						throw new Error(String.format("Velocity interpolation was too high (%f > %f)", velocity, maxV));
					}
					double minV = Math.min(s_velocities[i], s_velocities[i + 1]);
					if (velocity < minV && (minV - velocity) / minV > maxErr) {
						throw new Error(String.format("Velocity interpolation was too low (%f < %f)", velocity, minV));
					}
					double maxA = Math.max(s_angles[i], s_angles[i + 1]);
					if (angle > maxA && (angle - maxA) / maxA > maxErr) {
						throw new Error(String.format("Angle interpolation was too high (%f > %f)", angle, maxA));
					}
					double minA = Math.min(s_angles[i], s_angles[i + 1]);
					if (angle < minA && (minA - angle) / minA > maxErr) {
						throw new Error(String.format("Angle interpolation was too low (%f < %f)", angle, minA));
					}
				}
				System.out.println("All interpolation tests passing!!!!!!!");
			});
		}

		@Override
		protected double getShooterVelocity(double distance) {
			return interpolate(distance, s_velocities);
		}

		@Override
		protected double getHoodAngle(double distance) {
			return interpolate(distance, s_angles);
		}
	};
}