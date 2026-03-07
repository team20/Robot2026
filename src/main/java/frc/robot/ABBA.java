package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ABBA {
	/**
	 * Prevents a brownout by scaling down the power applied to a motor
	 * 
	 * @param power the power you want to apply if the robot can handle it
	 * @return a potentially reduced amount of power
	 */
	public static double preventBrownout(double power) {
		return preventBrownout(power, 0);
	}

	/**
	 * Prevents a brownout by scaling down the power applied to a motor
	 * 
	 * @param power the power you want to apply if the robot can handle it
	 * @param method which scaling factor you want to use (1 is piecwise, 2 is
	 *        cubic, 3 is exponential)
	 * @return a potentially reduced amount of power
	 */
	public static double preventBrownout(double power, int method) {
		return preventBrownout(power, method, RobotController.getBatteryVoltage());
	}

	/**
	 * Prevents a brownout by scaling down the power applied to a motor
	 * 
	 * @param power the power you want to apply if the robot can handle it
	 * @param method which scaling factor you want to use (1 is piecewise, 2 is
	 *        cubic, 3 is exponential)
	 * @param voltage the current voltage supply from the battery
	 * @return a potentially reduced amount of power
	 */
	public static double preventBrownout(double power, int method, double voltage) {
		return power * switch (method) {
			default -> piecewiseScalingFactor(voltage); // Original variation
			case 1 -> cubicScalingFactor(voltage); // Smoothest variation
			case 2 -> exponentialScalingFactor(voltage); // Smooth-ish but close to original
		};
	}

	/**
	 * Calculates how much to reduce power outputs based on the battery voltage
	 * 
	 * @param voltage the current voltage supply from the battery
	 * @return the factor to scale power outputs by
	 */
	private static double piecewiseScalingFactor(double voltage) {
		if (voltage < 7) {
			return 0.1;
		} else if (voltage < 8) {
			return 0.2;
		} else if (voltage < 9) {
			return 0.4;
		} else if (voltage < 10) {
			return 0.8;
		} else {
			return 1;
		}
	}

	/**
	 * Calculates how much to reduce power outputs based on the battery voltage
	 * 
	 * @param voltage the current voltage supply from the battery
	 * @return the factor to scale power outputs by
	 */
	private static double cubicScalingFactor(double voltage) {
		if (voltage < 6.5) {
			return 0.1;
		} else if (voltage < 11) {
			double t = voltage - 6.5;
			return (2.0 / 15.0 - 8.0 * t / 405.0) * t * t + .1;
		} else {
			return 1;
		}
	}

	/**
	 * Calculates how much to reduce power outputs based on the battery voltage
	 * 
	 * @param voltage the current voltage supply from the battery
	 * @return the factor to scale power outputs by
	 */
	private static double exponentialScalingFactor(double voltage) {
		if (voltage < 10) {
			return 0.9 * Math.pow(2.1, voltage - 10) + 0.1;
		} else {
			return 1;
		}
	}

	/**
	 * Gets a testing command which prints out whether the ABBA brownout prevention
	 * is behaving properly
	 * 
	 * @return the command
	 */
	public static Command testBrownoutPreventionCommand() {
		double[] niceLimits = new double[] { 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11,
				0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11,
				0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12,
				0.12, 0.12, 0.12, 0.13, 0.13, 0.13, 0.13, 0.13, 0.13, 0.14, 0.14, 0.14, 0.14, 0.15, 0.15, 0.15, 0.16,
				0.16, 0.17, 0.17, 0.18, 0.18, 0.19, 0.2, 0.2, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27, 0.29, 0.31, 0.4,
				0.4, 0.4, 0.42, 0.45, 0.48, 0.51, 0.54, 0.57, 0.6, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.82, 0.88, 0.94,
				1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
		return Commands.runOnce(() -> {
			int passing = 0;
			for (int i = 0; i < niceLimits.length; i++) {
				double voltage = 0.01 * i;
				double piecewise = preventBrownout(1, 1, voltage);
				double cubic = preventBrownout(1, 2, voltage);
				double exponential = preventBrownout(1, 3, voltage);
				if (piecewise <= niceLimits[i]) {
					passing++;
				}
				if (cubic <= niceLimits[i]) {
					passing++;
				}
				if (exponential <= niceLimits[i]) {
					passing++;
				}
			}
			System.out.printf("ABBA:\n%d Passing / %d Total\n", passing, 3 * niceLimits.length);
		});
	}
}