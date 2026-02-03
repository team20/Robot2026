package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ABBA {
	private static double m_current = -1;

	private static double getBatteryVoltage() {
		if (m_current > 0) {
			return BatterySim.calculateDefaultBatteryLoadedVoltage(m_current);
		} else {
			return RobotController.getBatteryVoltage();
		}
	}

	// If you think that you want to run at 7 out of 12 volts,
	// this will return 7 when the battery is running great, or something less than
	// that if it is not.
	// example: voltage-based control: m_voltage =
	// ABBA.preventBrownout(requestedVoltage, 12);
	// percentage-based control: m_percent = ABBA.preventBrownout(requestedPercent,
	// 100)
	public static double preventBrownout(double requestedPower, double normalMaxPower, boolean smooth) {
		double maxVoltage = smooth ? getMaxVoltageSmooth() : getMaxVoltage();
		double maxScaled = maxVoltage * normalMaxPower / 12;
		double absPower = Math.min(Math.abs(requestedPower), maxScaled);
		return absPower * Math.signum(requestedPower);
	}

	public static double preventBrownout(double requestedPower, double normalMaxPower) {
		return preventBrownout(requestedPower, normalMaxPower, false);
	}

	public static double getMaxVoltage() {
		double voltage = getBatteryVoltage();
		if (voltage < 7) {
			return 1.5;
		} else if (voltage < 8) {
			return 2.5;
		} else if (voltage < 9) {
			return 5;
		} else if (voltage < 10) {
			return 9.5;
		} else {
			return voltage;
		}
	}

	public static double getMaxVoltageSmooth() {
		double voltage = getBatteryVoltage();
		return Math.min(.3 * Math.pow(1.4, voltage), voltage);
	}

	public static void setTestingCurrent(double current) {
		m_current = current;
	}

	public static Command testBrownoutPreventionCommand() {
		return Commands.runOnce(() -> {
			setTestingCurrent(200);
			if (preventBrownout(10, 12) > 5) {
				throw new Error("ABBA didn't limit the power in normal mode");
			}
			if (preventBrownout(10, 12, true) > 5) {
				throw new Error("ABBA didn't limit the power in smooth mode");
			}
			if (getMaxVoltage() > 5) {
				throw new Error("ABBA didn't limit the max voltage in normal mode");
			}
			if (getMaxVoltageSmooth() > 5) {
				throw new Error("ABBA didn't limit the max voltage in smooth mode");
			}
			System.out.println("All ABBA tests passing!");
		});
	}
}
