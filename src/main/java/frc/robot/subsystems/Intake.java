package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	private final SparkMax m_intakeWheels;
	private final SparkMaxConfig m_wheelConfig;

	// Set up motor & initialize other subsystem aspects
	public Intake
	// Instantiate motor & motor configurations
	m_intakeWheels = new SparkMax(kIntakeWheelsPort, MotorType.kBrushless);m_wheelConfig=new SparkMaxConfig();

	// TODO: Check configuration of motors
	m_wheelConfig.smartCurrentLimit(kWheelSmartCurrentLimit);m_wheelConfig.secondaryCurrentLimit(kWheelSecondaryCurrentLimit);m_wheelConfig.idleMode(IdleMode.kBrake);m_wheelConfig.inverted(kWheelInvert);

	m_intakeWheels.configure(m_wheelConfig,ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);
	}

	/**
	 * Set power of motor driving intake roller wheels.
	 * 
	 * @param speed
	 */
	public void setWheelPower(double power) {
		m_intakeWheels.set(power);
	}
}