package frc.robot.subsystems;

import static frc.robot.Constants.Subsystems.ClimberConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	public final SparkMax m_climber;
	public final SparkMaxConfig m_config;

	public Climber() {
		m_climber = new SparkMax(kClimberPort, MotorType.kBrushless);
		m_config = new SparkMaxConfig();

		// TODO: Update motor configuration
		m_config.smartCurrentLimit(kSmartCurrentLimit);
		m_config.secondaryCurrentLimit(kSecondaryCurrentLimit);
		m_config.inverted(kInvert);

		m_climber.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	/**
	 * Applies a given power to the motor driving the climber's winch.
	 * 
	 * @param power (0.0 to 1.0)
	 */
	public void run(double power) {
		m_climber.set(power);
	}
}
