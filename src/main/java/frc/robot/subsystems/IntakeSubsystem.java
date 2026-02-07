package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
// import motor stuff
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsystem {
	private SparkMax m_motor;

	// config
	SparkMaxConfig config = new SparkMaxConfig();
	// set config values
	config.inverted(IntakeConstants.kInvert);
	config.smartCurrentLimit(IntakeConstants.kSmartCurrentLimit);

	public IntakeSubsystem() {
		// can id is a placeholder; replace later.
		m_motor = new SparkMax(10, MotorType.kBrushless);
		// Apply config to SparkMax
		m_motor.apply(config);
	}

	// set speed for motor
	public void setSpeed(double speed) {
		m_motor.set(speed);
	}
}
