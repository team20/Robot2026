package robot.utilities;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import robot.PhysicalBot;

/**
 * A class which stores constants for the swerve drive on a per robot basis.
 */
public class BotConstants implements Map.Entry<PhysicalBot, BotConstants> {
	/**
	 * Keep track of which robot these constants apply to.
	 */
	private final PhysicalBot m_bot;
	/**
	 * Stores the maximum voltage for drive motors.
	 */
	public double kTeleopMaxVoltage;
	/**
	 * Stores the maximum voltage for steer motors.
	 */
	public double kTeleopMaxTurnVoltage;
	/**
	 * Stores the gear ratio for drive motors.
	 */
	public double kDriveGearRatio;
	/**
	 * Stores the gear ratio for steer motors.
	 */
	public double kSteerGearRatio;
	/**
	 * Stores the wheel diameter in meters.
	 */
	public double kWheelDiameter;

	/**
	 * Creates a new set of constants for the specified robot.
	 * 
	 * @param bot robot which the constants apply to
	 */
	public BotConstants(PhysicalBot bot) {
		m_bot = bot;
	}

	/**
	 * Sets the maximum voltage for the drive motors.
	 * 
	 * @param voltage voltage in volts
	 * @return constants for this robot
	 */
	public BotConstants setTeleopMaxVoltage(double voltage) {
		kTeleopMaxVoltage = voltage;
		return this;
	}

	/**
	 * Sets the maximum voltage for the steer motors.
	 * 
	 * @param voltage voltage in volts
	 * @return contants for this robot
	 */
	public BotConstants setTeleopMaxTurnVoltage(double voltage) {
		kTeleopMaxTurnVoltage = voltage;
		return this;
	}

	/**
	 * Sets the gear ratio for the drive motors.
	 * 
	 * @param ratio ratio of the drive motor gear train
	 * @return constants for this robot
	 */
	public BotConstants setDriveGearRatio(double ratio) {
		kDriveGearRatio = ratio;
		return this;
	}

	/**
	 * Sets the gear ratio for the steer motors.
	 * 
	 * @param ratio ratio of the steer motor gear train
	 * @return constants for this robot
	 */
	public BotConstants setSteerGearRatio(double ratio) {
		kSteerGearRatio = ratio;
		return this;
	}

	/**
	 * Sets the wheel diameter and converts from inches to meters.
	 * 
	 * @param diameter diameter in inches
	 * @return constants for this robot
	 */
	public BotConstants setWheelDiameter(double diameter) {
		kWheelDiameter = Units.inchesToMeters(diameter);
		return this;
	}

	/**
	 * Gets the wheel circumference in meters.
	 * 
	 * @return wheel circumference in meters
	 */
	public double getWheelCircumference() {
		return kWheelDiameter * Math.PI;
	}

	/**
	 * Gets the distance the robot travels for each rotation of the wheels.
	 * 
	 * @return ratio of meters per each rotation
	 */
	public double getMetersPerMotorRotation() {
		return kWheelDiameter * Math.PI / kDriveGearRatio;
	}

	/**
	 * Allows the constants to be stored in a map by key.
	 * 
	 * @return robot which these constants apply to
	 */
	@Override
	public PhysicalBot getKey() {
		return m_bot;
	}

	/**
	 * Allows the constants to be looked up by key in a map.
	 * 
	 * @return constants for this robot
	 */
	@Override
	public BotConstants getValue() {
		return this;
	}

	/**
	 * Sets these constants to be identical to those of another robot.
	 * 
	 * @return constants for this robot
	 */
	@Override
	public BotConstants setValue(BotConstants value) {
		kTeleopMaxVoltage = value.kTeleopMaxVoltage;
		kTeleopMaxTurnVoltage = value.kTeleopMaxTurnVoltage;
		kDriveGearRatio = value.kDriveGearRatio;
		kSteerGearRatio = value.kSteerGearRatio;
		kWheelDiameter = value.kWheelDiameter;
		return this;
	}
}
