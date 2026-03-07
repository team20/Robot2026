package frc.robot;

import edu.wpi.first.wpilibj.RobotController;

public class ABBA {
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
