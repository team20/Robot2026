package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClampedP {
	/**
	 * Calculates a clamped p controller output power. The power can be any range
	 * you like, such as 0-1 or 0-12.
	 * 
	 * @param error The calculated error value (current value minus setpoint)
	 * @param minPower The minimum output power to overcome friction
	 * @param maxPower The maximum output power for long distance travel
	 * @param maxErr The distance at which to start slowing down
	 * @param tolerance The tolerance at which to cut the power
	 * @return The calculated value
	 */
	public static double clampedP(double error, double minPower, double maxPower, double maxErr, double tolerance) {
		if (Math.abs(error) < tolerance) {
			return 0;
		}
		double p = (maxPower - minPower) / (maxErr - tolerance);
		double power = minPower + (Math.abs(error) - tolerance) * p;
		if (power > maxPower) {
			return -Math.signum(error) * maxPower;
		}
		return -Math.signum(error) * power;
	}

	public static Command testCommand() {
		return Commands.runOnce(() -> {
			double minPower = 1;
			double maxPower = 50;
			double maxErr = 10;
			double tolerance = 1;
			int passing = 0;
			int failing = 0;
			double[] testErrors = new double[] { -25, -10, -5.5, -1, -0.5, 0, 0.5, 1, 5.5, 10, 25 };
			double[] testOutput = new double[] { 50, 50, 25.5, 1, 0, 0, 0, -1, -25.5, -50, -50 };
			for (int i = 0; i < testErrors.length; i++) {
				double output = clampedP(testErrors[i], minPower, maxPower, maxErr, tolerance);
				if (output == testOutput[i]) {
					passing++;
				} else {
					failing++;
					System.out.printf("Got %f but expected %f\n", output, testOutput[i]);
				}
			}
			System.out.printf("ClampedP:\n%d Passing / %d Failing\n", passing, failing);
		});
	}
}