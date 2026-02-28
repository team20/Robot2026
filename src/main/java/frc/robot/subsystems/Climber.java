package frc.robot.subsystems;

import static frc.robot.Constants.Subsystems.ClimberConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

	private static Climber s_climber;

	public final SparkMax m_motor;
	public final SparkMaxConfig m_config;

	public Climber() {
		m_motor = new SparkMax(kClimberPort, MotorType.kBrushless);

		m_config = new SparkMaxConfig();

		// TODO: Update motor configuration
		m_config.smartCurrentLimit(kSmartCurrentLimit);
		m_config.secondaryCurrentLimit(kSecondaryCurrentLimit);
		m_config.inverted(kInvert);

		m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		if (s_climber == null)
			s_climber = this;
		else
			throw new Error("Climber already instantiated");
	}

	/**
	 * Applies a given power to the motor driving the climber's winch.
	 * 
	 * @param power (0.0 to 1.0)
	 */
	public static void setPower(double power) {
		s_climber.m_motor.set(power);
	}

	public static Climber getClimber() {
		return s_climber;
	}

	public static void stop() {
		s_climber.m_motor.set(0);
	}

	public static void setVoltage(double voltage) {
		s_climber.m_motor.setVoltage(voltage);
	}

}