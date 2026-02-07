package frc.robot.subsystems;

<<<<<<< HEAD
import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	private final SparkMax m_intakeWheels;
	private final SparkMaxConfig m_wheelConfig;

	// Set up motor & initialize other subsystem aspects
	public IntakeSubsystem() {
		// Instantiate motor & motor configurations
		m_intakeWheels = new SparkMax(kIntakeWheelsPort, MotorType.kBrushless);
		m_wheelConfig = new SparkMaxConfig();

		// TODO: Check configuration of motors
		m_wheelConfig.smartCurrentLimit(kWheelSmartCurrentLimit);
		m_wheelConfig.secondaryCurrentLimit(kWheelSecondaryCurrentLimit);
		m_wheelConfig.idleMode(IdleMode.kBrake);
		m_wheelConfig.inverted(kWheelInvert);

		m_intakeWheels.configure(m_wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	/**
	 * Set power of motor driving intake roller wheels.
	 * 
	 * @param speed
	 */
	public void setWheelPower(double power) {
		m_intakeWheels.set(power);
=======
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
>>>>>>> 4a47a4e (67)
	}
}
