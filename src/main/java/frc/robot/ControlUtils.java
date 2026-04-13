package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControlUtils {
	public static class ScaledJoystick {
		private final DoubleSupplier m_xSupplier;
		private final DoubleSupplier m_ySupplier;
		private final double m_deadzone;
		private final double m_intensity;
		private double m_x;
		private double m_y;

		/**
		 * Applies a deadzone and scales joystick input to make it smoother.
		 * 
		 * @param x a supplier of the raw joystick x value
		 * @param y a supplier of the raw joystick y value
		 * @param deadzone the size of the deadzone
		 */
		public ScaledJoystick(DoubleSupplier x, DoubleSupplier y, double deadzone) {
			this(x, y, deadzone, 1);
		}

		/**
		 * Applies a deadzone and scales joystick input to make it smoother.
		 * 
		 * @param x a supplier of the raw joystick x value
		 * @param y a supplier of the raw joystick y value
		 * @param deadzone the size of the deadzone
		 * @param intensity a number from 0 to 1 of how sensitive the joystick is
		 */
		public ScaledJoystick(DoubleSupplier x, DoubleSupplier y, double deadzone, double intensity) {
			m_xSupplier = x;
			m_ySupplier = y;
			m_deadzone = deadzone;
			m_intensity = intensity;
		}

		/**
		 * Update the current x and y output
		 */
		public void update() {
			double x = m_xSupplier.getAsDouble();
			double y = m_ySupplier.getAsDouble();
			double magnitude = Math.min(Math.hypot(x, y), 1);
			double direction = Math.atan2(y, x);
			double deadzoned = magnitude <= m_deadzone ? 0 : magnitude;
			double intense = 2 * Math.asin(deadzoned) / Math.PI;
			double smooth = deadzoned * deadzoned * deadzoned;
			magnitude = m_intensity * intense + (1 - m_intensity) * smooth;
			m_x = magnitude * Math.cos(direction);
			m_y = magnitude * Math.sin(direction);
		}

		/**
		 * Gets the current x output
		 * 
		 * @return x value
		 */
		public double getX() {
			return m_x;
		}

		/**
		 * Gets the current y output
		 * 
		 * @return y value
		 */
		public double getY() {
			return m_y;
		}

		public static Command testCommand() {
			return Commands.runOnce(() -> {
				ScaledJoystick joystick = new ScaledJoystick(() -> .5, () -> .5, .05);
				joystick.update();
				System.out.printf("Joystick output: (%f, %f)\n", joystick.getX(), joystick.getY());
			});
		}
	}

	public static class ABBA {
		public static double preventBrownout(double power) {
			double voltage = RobotController.getBatteryVoltage();
			if (voltage < 7) {
				return power * 0.1;
			} else if (voltage < 8) {
				return power * 0.2;
			} else if (voltage < 9) {
				return power * 0.4;
			} else if (voltage < 10) {
				return power * 0.8;
			} else {
				return power;
			}
		}
	}

	public static class ClampedP {
		public record Constants(double minPower, double maxPower, double maxErr, double tolerance) {
		};

		/**
		 * Calculates a clamped p controller output power. The power can be any range
		 * you like, such as 0-1 or 0-12.
		 * 
		 * @param error The calculated error value (current value minus setpoint)
		 * @param constants The constants to use
		 * @return The calculated value
		 */
		public static double clampedP(double error, Constants constants) {
			return clampedP(
					error, constants.minPower(), constants.maxPower(), constants.maxErr(), constants.tolerance());
		}

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
}