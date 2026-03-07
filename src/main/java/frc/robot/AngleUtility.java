package frc.robot;

public class AngleUtility {
	public static double minDifference(double a, double b) {
		double difference = Math.abs(a - b);
		return Math.min(difference, 360 - difference);
	}
}
