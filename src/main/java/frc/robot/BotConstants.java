package frc.robot;

import java.util.Map;

import edu.wpi.first.math.util.Units;

public class BotConstants implements Map.Entry<PhysicalBot, BotConstants> {
	private final PhysicalBot m_bot;
	public double kTeleopMaxVoltage;
	public double kTeleopMaxTurnVoltage;
	public double kDriveGearRatio;
	public double kSteerGearRatio;
	public double kWheelDiameter;

	public BotConstants(PhysicalBot bot) {
		m_bot = bot;
	}

	public BotConstants setTeleopMaxVoltage(double voltage) {
		kTeleopMaxVoltage = voltage;
		return this;
	}

	public BotConstants setTeleopMaxTurnVoltage(double voltage) {
		kTeleopMaxTurnVoltage = voltage;
		return this;
	}

	public BotConstants setDriveGearRatio(double ratio) {
		kDriveGearRatio = ratio;
		return this;
	}

	public BotConstants setSteerGearRatio(double ratio) {
		kSteerGearRatio = ratio;
		return this;
	}

	public BotConstants setWheelDiameter(double diameter) {
		kWheelDiameter = Units.inchesToMeters(diameter);
		return this;
	}

	public double getWheelCircumference() {
		return kWheelDiameter * Math.PI;
	}

	public double getMetersPerMotorRotation() {
		return kWheelDiameter * Math.PI / kDriveGearRatio;
	}

	@Override
	public PhysicalBot getKey() {
		return m_bot;
	}

	@Override
	public BotConstants getValue() {
		return this;
	}

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
