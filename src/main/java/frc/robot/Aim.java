package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.ToDoubleFunction;

import edu.wpi.first.math.util.Units;

public abstract class Aim {

	public abstract double getShooterVelocity(double distance);

	public abstract double getHoodAngle(double distance);

	public abstract double getShotAirtime(double distance);

	public static class Linear extends Aim {

		// Represents the state of the Hood/Flywheel + the airtime the ball will have
		public record ShooterPreset(double angle, double rpm, double airtime) {
		}

		/**
		 * Mapping from requested distance from the turret the center of the hub ==> to
		 * their preset {@code ShooterPreset}. Tree map automatically takes care of
		 * sorting these by their keys (distances) and when the angles or speeds are
		 * streamed, they come out in order
		 * 
		 * @see java.util.TreeMap
		 **/

		private static final TreeMap<Double, ShooterPreset> s_presets = new TreeMap<>(Map.of(
				-500.0, new ShooterPreset(99, 2000, 1.1),
				3.17, new ShooterPreset(99, 2000, 1.1),
				6.23, new ShooterPreset(113, 2100, 1.1),
				9.1, new ShooterPreset(122, 2300, 1.2),
				13.1, new ShooterPreset(137, 2535, 1.4),
				18.0, new ShooterPreset(137, 2850, 1.4),
				500.0, new ShooterPreset(137, 3000, 1.4)));

		private static double interpolate(double a, double b, double t) {
			return (1 - t) * a + t * b;
		}

		private static int getIndex(double distance) {
			List<Double> distances = s_presets.keySet().stream().toList();
			for (int i = 0; i < distances.size() - 1; i++) {
				if (distances.get(i) <= distance && distances.get(i + 1) > distance) {
					return i;
				}
			}
			return 0;
		}

		private static double interpolate(double distance, List<Double> list) {
			int index = getIndex(distance);
			if (index < 0) {
				return -1;
			} else {
				List<Double> distances = s_presets.keySet().stream().toList();

				double a = list.get(index);
				double b = list.get(index + 1);
				double delta = distances.get(index + 1) - distances.get(index);
				double t = (distance - distances.get(index)) / delta;
				return interpolate(a, b, t);
			}
		}

		@Override
		public double getShooterVelocity(double distance) {
			List<Double> velocities = s_presets.values().stream().map(p -> p.rpm).toList();
			return interpolate(distance, velocities);
		}

		@Override
		public double getHoodAngle(double distance) {
			List<Double> angles = s_presets.values().stream().map(p -> p.angle).toList();
			return interpolate(distance, angles);
		}

		@Override
		public double getShotAirtime(double distance) {
			List<Double> airtimes = s_presets.values().stream().map(p -> p.airtime).toList();
			return interpolate(distance, airtimes);
		}

	}

	public static class AirtimeRegression extends Aim {
		public record ShooterPreset(double distance, double angle, double rpm) {
		}

		private static final ShooterPreset[] s_presets = new ShooterPreset[] {
				new ShooterPreset(-500, 99, 2000),
				new ShooterPreset(3.17, 99, 2000),
				new ShooterPreset(6.23, 113, 2100),
				new ShooterPreset(9.1, 122, 2300),
				new ShooterPreset(13.1, 137, 2535),
				new ShooterPreset(18.0, 137, 2850),
				new ShooterPreset(500, 137, 3000) };

		private static double interpolate(double a, double b, double t) {
			return (1 - t) * a + t * b;
		}

		private static int getIndex(double distance) {
			for (int i = 0; i < s_presets.length - 1; i++) {
				if (s_presets[i].distance() <= distance && s_presets[i + 1].distance() > distance) {
					return i;
				}
			}
			return 0;
		}

		private static double interpolate(double distance, ToDoubleFunction<ShooterPreset> accessor) {
			int index = getIndex(distance);
			if (index < 0) {
				return -1;
			} else {
				double a = accessor.applyAsDouble(s_presets[index]);
				double b = accessor.applyAsDouble(s_presets[index + 1]);
				double delta = s_presets[index + 1].distance() - s_presets[index].distance();
				double t = (distance - s_presets[index].distance()) / delta;
				return interpolate(a, b, t);
			}
		}

		@Override
		public double getShooterVelocity(double distance) {
			return interpolate(distance, ShooterPreset::rpm);
		}

		@Override
		public double getHoodAngle(double distance) {
			return interpolate(distance, ShooterPreset::angle);
		}

		@Override
		public double getShotAirtime(double distance) {
			double rpm = getShooterVelocity(distance);
			double angle = getHoodAngle(distance);
			double b = (rpm / 116 + 4.5) * Math.sin(Units.degreesToRadians(122 - angle / 2.46));
			return (b + Math.sqrt(b * b - 277)) / 32.2;
		}
	}
}