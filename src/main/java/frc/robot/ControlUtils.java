package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControlUtils {
	public static class ScaledJoystick {
		public enum IntensityPresets {
			INTENSE(-0.5),
			STANDARD(0),
			SMOOTH(1),
			TESTING(5);

			private final double m_smoothness;

			private IntensityPresets(double smoothness) {
				m_smoothness = smoothness;
			}

			public double getSmoothness() {
				return m_smoothness;
			}
		}

		private final DoubleSupplier m_xSupplier;
		private final DoubleSupplier m_ySupplier;
		private final double m_deadzone;
		private final double m_smoothness;
		private double m_x;
		private double m_y;
		private boolean m_staleX = true;
		private boolean m_staleY = true;

		/**
		 * Applies a deadzone and scales joystick input to make it smoother.
		 * 
		 * @param x a supplier of the raw joystick x value
		 * @param y a supplier of the raw joystick y value
		 * @param deadzone the size of the deadzone
		 */
		public ScaledJoystick(DoubleSupplier x, DoubleSupplier y, double deadzone) {
			this(x, y, deadzone, IntensityPresets.STANDARD);
		}

		/**
		 * Applies a deadzone and scales joystick input to make it smoother.
		 * 
		 * @param x a supplier of the raw joystick x value
		 * @param y a supplier of the raw joystick y value
		 * @param deadzone the size of the deadzone
		 * @param smoothness an intensity preset
		 */
		public ScaledJoystick(DoubleSupplier x, DoubleSupplier y, double deadzone, IntensityPresets preset) {
			m_xSupplier = x;
			m_ySupplier = y;
			m_deadzone = deadzone;
			m_smoothness = preset.getSmoothness();
		}

		/**
		 * Applies a deadzone and scales joystick input to make it smoother.
		 * 
		 * @param x a supplier of the raw joystick x value
		 * @param y a supplier of the raw joystick y value
		 * @param deadzone the size of the deadzone
		 * @param smoothness a number from -1 to infinity of how sensitive the joystick
		 *        is
		 */
		public ScaledJoystick(DoubleSupplier x, DoubleSupplier y, double deadzone, double smoothness) {
			m_xSupplier = x;
			m_ySupplier = y;
			m_deadzone = deadzone;
			m_smoothness = smoothness;
		}

		/**
		 * Update the current x and y output
		 */
		private void update() {
			m_staleX = m_staleY = false;
			double x = m_xSupplier.getAsDouble();
			double y = m_ySupplier.getAsDouble();
			double magnitude = Math.min(Math.hypot(x, y), 1);
			if (magnitude <= m_deadzone) {
				m_x = 0;
				m_y = 0;
			} else {
				double intense = 0.857956 * Math.asin(0.91901 * magnitude);
				double smooth = magnitude * magnitude;
				double scaled_magnitude = intense * Math.pow(smooth / intense, m_smoothness);
				m_x = x / magnitude * scaled_magnitude;
				m_y = y / magnitude * scaled_magnitude;
			}
		}

		/**
		 * Gets the current x output
		 * 
		 * @return x value
		 */
		public double getX() {
			if (m_staleX) {
				update();
			}
			m_staleX = true;
			return m_x;
		}

		/**
		 * Gets the current y output
		 * 
		 * @return y value
		 */
		public double getY() {
			if (m_staleY) {
				update();
			}
			m_staleY = true;
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