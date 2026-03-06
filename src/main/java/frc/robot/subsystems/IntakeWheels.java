package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.IntakeConstants;

public class IntakeWheels extends SubsystemBase {
	private static IntakeWheels s_theIntake;
	private final SparkMax m_intakeWheels;
	private final SparkMaxConfig m_wheelConfig;

	// Set up motor & initialize other subsystem aspects
	public IntakeWheels() {
		// Instantiate motor & motor configurations
		m_intakeWheels = new SparkMax(IntakeConstants.kIntakeWheelsPort, MotorType.kBrushless);
		m_wheelConfig = new SparkMaxConfig();

		// TODO: Check configuration of motors
		m_wheelConfig.smartCurrentLimit(IntakeConstants.kWheelSmartCurrentLimit);
		m_wheelConfig.secondaryCurrentLimit(IntakeConstants.kWheelSecondaryCurrentLimit);
		m_wheelConfig.idleMode(IdleMode.kBrake);
		m_wheelConfig.inverted(IntakeConstants.kWheelInvert);

		m_intakeWheels.configure(m_wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		if (s_theIntake == null) {
			s_theIntake = this;
		} else {
			throw new Error("Intake already instantiated");
		}
	}

	public static IntakeWheels getIntake() {
		return s_theIntake;
	}

	/**
	 * Set power of motor driving intake roller wheels.
	 * 
	 * @param speed
	 */
	public static void setWheelPower(double power) {
		s_theIntake.m_intakeWheels.set(power);
	}

	public static void stopWheel() {
		s_theIntake.m_intakeWheels.stopMotor();
	}
}