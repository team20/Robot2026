package frc.robot;

import java.util.function.ToDoubleFunction;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AngularPositionCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.Hood;

public abstract class Aim {
	public record InterpolationPoint(double distance, double angle, int rpm) {
	}

	public abstract double getShooterVelocity(double distance);

	public abstract double getHoodAngle(double distance);

	public Command getAimCommand(double distance) {
		return Commands.parallel(
				new ShooterCommands.RunAtDynamicRPM(getShooterVelocity(distance)),
				new AngularPositionCommands.SetAngleHardware(Hood.getHood(), getHoodAngle(distance)),
				new AngularPositionCommands.SettleAngle(Hood.getHood(), Hood.getConstants().tolerance()));
	}

	public static class Linear extends Aim {
		private static final InterpolationPoint[] s_points = new InterpolationPoint[] {
				new InterpolationPoint(-500, 99, 2000),
				new InterpolationPoint(5.333, 99, 2000),
				new InterpolationPoint(9.1, 122, 2300),
				new InterpolationPoint(13.1, 137, 2535),
				new InterpolationPoint(18, 137, 2850),
				new InterpolationPoint(500, 137, 3000)
		};

		private static double interpolate(double a, double b, double t) {
			return (1 - t) * a + t * b;
		}

		private static int getIndex(double distance) {
			for (int i = 0; i < s_points.length - 1; i++) {
				if (s_points[i].distance() <= distance && s_points[i + 1].distance() > distance) {
					return i;
				}
			}
			return -1;
		}

		private static double interpolate(double distance, ToDoubleFunction<InterpolationPoint> accessor) {
			int index = getIndex(distance);
			if (index < 0) {
				return -1;
			} else {
				double a = accessor.applyAsDouble(s_points[index]);
				double b = accessor.applyAsDouble(s_points[index + 1]);
				double delta = s_points[index + 1].distance() - s_points[index].distance();
				double t = (distance - s_points[index].distance()) / delta;
				return interpolate(a, b, t);
			}
		}

		@Override
		public double getShooterVelocity(double distance) {
			return interpolate(distance, InterpolationPoint::rpm);
		}

		@Override
		public double getHoodAngle(double distance) {
			return interpolate(distance, InterpolationPoint::angle);
		}

		public static Command testCommand() {
			return Commands.runOnce(() -> {
				Aim aimer = new Interpolation();
				for (int i = 0; i < s_points.length - 1; i++) {
					double middle = (s_points[i].distance() + s_points[i + 1].distance()) / 2;
					double velocity = aimer.getShooterVelocity(middle);
					double angle = aimer.getHoodAngle(middle);
					double maxV = Math.max(s_points[i].rpm(), s_points[i + 1].rpm());
					double maxErr = 0.01;
					if (velocity > maxV && (velocity - maxV) / maxV > maxErr) {
						throw new Error(String.format("Velocity interpolation was too high (%f > %f)", velocity, maxV));
					}
					double minV = Math.min(s_points[i].rpm(), s_points[i + 1].rpm());
					if (velocity < minV && (minV - velocity) / minV > maxErr) {
						throw new Error(String.format("Velocity interpolation was too low (%f < %f)", velocity, minV));
					}
					double maxA = Math.max(s_points[i].angle(), s_points[i + 1].angle());
					if (angle > maxA && (angle - maxA) / maxA > maxErr) {
						throw new Error(String.format("Angle interpolation was too high (%f > %f)", angle, maxA));
					}
					double minA = Math.min(s_points[i].angle(), s_points[i + 1].angle());
					if (angle < minA && (minA - angle) / minA > maxErr) {
						throw new Error(String.format("Angle interpolation was too low (%f < %f)", angle, minA));
					}
				}
				System.out.println("All interpolation tests passing!!!!!!!");
			});
		}
	}

	public static class Interpolation extends Aim {
		private static final InterpolationPoint[] s_points = new InterpolationPoint[] {
				new InterpolationPoint(-500, 99, 2000),
				new InterpolationPoint(5.333, 99, 2000),
				new InterpolationPoint(9.1, 122, 2300),
				new InterpolationPoint(13.1, 137, 2535),
				new InterpolationPoint(18, 137, 2850),
				new InterpolationPoint(500, 137, 3000)
		};

		private static double interpolate(double a, double b, double da, double db, double t) {
			double result = 0;
			result += ((2 * t - 3) * t * t + 1) * a;
			result += ((t - 2) * t + 1) * t * da;
			result += (3 - 2 * t) * t * t * b;
			result += (t - 1) * t * t * db;
			return result;
		}

		private static int getIndex(double distance) {
			for (int i = 0; i < s_points.length - 1; i++) {
				if (s_points[i].distance() <= distance && s_points[i + 1].distance() > distance) {
					return i;
				}
			}
			return -1;
		}

		private static double interpolate(double distance, ToDoubleFunction<InterpolationPoint> accessor) {
			int index = getIndex(distance);
			if (index < 0) {
				return -1;
			} else {
				int end = s_points.length - 2;
				double a = accessor.applyAsDouble(s_points[index]);
				double b = accessor.applyAsDouble(s_points[index + 1]);
				double da;
				double db;
				if (index == 0) {
					da = (accessor.applyAsDouble(s_points[1]) - accessor.applyAsDouble(s_points[0]))
							/ (s_points[1].distance() - s_points[0].distance());
					db = ((accessor.applyAsDouble(s_points[2]) - accessor.applyAsDouble(s_points[1]))
							/ (s_points[2].distance() - s_points[1].distance()) + da) / 2;
				} else if (index == end) {
					db = (accessor.applyAsDouble(s_points[end + 1]) - accessor.applyAsDouble(s_points[end]))
							/ (s_points[end + 1].distance() - s_points[end].distance());
					da = ((accessor.applyAsDouble(s_points[end]) - accessor.applyAsDouble(s_points[end - 1]))
							/ (s_points[end].distance() - s_points[end - 1].distance()) + db) / 2;
				} else {
					double d = (b - a) / (s_points[index + 1].distance() - s_points[index].distance());
					da = ((a - accessor.applyAsDouble(s_points[index - 1]))
							/ (s_points[index].distance() - s_points[index - 1].distance()) + d) / 2;
					db = ((accessor.applyAsDouble(s_points[index + 2]) - b)
							/ (s_points[index + 2].distance() - s_points[index + 1].distance()) + d) / 2;
				}
				double delta = s_points[index + 1].distance() - s_points[index].distance();
				da *= delta;
				db *= delta;
				double t = (distance - s_points[index].distance()) / delta;
				return interpolate(a, b, da, db, t);
			}
		}

		public static Command testCommand() {
			return Commands.runOnce(() -> {
				Aim aimer = new Interpolation();
				for (int i = 0; i < s_points.length - 1; i++) {
					double middle = (s_points[i].distance() + s_points[i + 1].distance()) / 2;
					double velocity = aimer.getShooterVelocity(middle);
					double angle = aimer.getHoodAngle(middle);
					double maxV = Math.max(s_points[i].rpm(), s_points[i + 1].rpm());
					double maxErr = 0.01;
					if (velocity > maxV && (velocity - maxV) / maxV > maxErr) {
						throw new Error(String.format("Velocity interpolation was too high (%f > %f)", velocity, maxV));
					}
					double minV = Math.min(s_points[i].rpm(), s_points[i + 1].rpm());
					if (velocity < minV && (minV - velocity) / minV > maxErr) {
						throw new Error(String.format("Velocity interpolation was too low (%f < %f)", velocity, minV));
					}
					double maxA = Math.max(s_points[i].angle(), s_points[i + 1].angle());
					if (angle > maxA && (angle - maxA) / maxA > maxErr) {
						throw new Error(String.format("Angle interpolation was too high (%f > %f)", angle, maxA));
					}
					double minA = Math.min(s_points[i].angle(), s_points[i + 1].angle());
					if (angle < minA && (minA - angle) / minA > maxErr) {
						throw new Error(String.format("Angle interpolation was too low (%f < %f)", angle, minA));
					}
				}
				System.out.println("All interpolation tests passing!!!!!!!");
			});
		}

		@Override
		public double getShooterVelocity(double distance) {
			return interpolate(distance, InterpolationPoint::rpm);
		}

		@Override
		public double getHoodAngle(double distance) {
			return interpolate(distance, InterpolationPoint::distance);
		}
	};
}
