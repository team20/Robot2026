package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class Aim {

	// Represents the state of the Hood/Flywheel + the airtime the ball will have
	public record ShooterPreset(double angle, double rpm, double airtime) {
	}

	/**
	 * Mapping from requested distance (in ft) from turret to center of hub ==> to
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
			9.1, new ShooterPreset(128, 2440, 1.2),
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

	public static double getShooterVelocity(double distance) {
		List<Double> velocities = s_presets.values().stream().map(p -> p.rpm).toList();
		return interpolate(distance - 1, velocities);
	}

	public static double getHoodAngle(double distance) {
		List<Double> angles = s_presets.values().stream().map(p -> p.angle).toList();
		return interpolate(distance - 1, angles);
	}

	public static double getShotAirtime(double distance) {
		List<Double> airtimes = s_presets.values().stream().map(p -> p.airtime).toList();
		return interpolate(distance - 1, airtimes);// + .75; // TODO: Update to be a constant once fully tuned
	}

}
