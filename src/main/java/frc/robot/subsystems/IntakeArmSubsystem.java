package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArmSubsystem extends SubsystemBase {
	private final SparkMax m_intakeArm;
	private final SparkMaxConfig m_armConfig;

	// Set up motor & initialize other subsystem aspects
	public IntakeArmSubsystem() {
		// Instantiate motor & motor configuration
		m_intakeArm = new SparkMax(kIntakeArmPort, MotorType.kBrushless);
		m_armConfig = new SparkMaxConfig();

		// TODO: Check configuration of motors
		m_armConfig.smartCurrentLimit(kArmSmartCurrentLimit);
		m_armConfig.secondaryCurrentLimit(kArmSecondaryCurrentLimit);
		m_armConfig.idleMode(IdleMode.kBrake);
		m_armConfig.inverted(kArmInvert);

		m_intakeArm.configure(m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	/**
	 * Set power of motor driving intake deployment arm.
	 * 
	 * @param speed
	 */
	public void setArmPower(double power) {
		m_intakeArm.set(power);
	}

	// We are NOT using an absolute encoder! We are using a limit switch!! Change
	// this method!!!
	public double getArmAngle() {
		return m_intakeArm.getAbsoluteEncoder().getPosition() * 360;
	}
}