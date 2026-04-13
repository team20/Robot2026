package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.IntakeConstants;

public class IntakeExtraArm extends SubsystemBase {
	private static IntakeExtraArm s_theExtraIntakeArm;
	private final SparkMax m_intakeExtraArm;
	private final SparkMaxConfig m_armConfig;

	// Set up motor & initialize other subsystem aspects
	public IntakeExtraArm() {
		// Instantiate motor & motor configurations
		m_intakeExtraArm = new SparkMax(IntakeConstants.kIntakeExtraArmPort, MotorType.kBrushless);
		m_armConfig = new SparkMaxConfig();

		m_armConfig.smartCurrentLimit(IntakeConstants.kExtraArmSmartCurrentLimit);
		m_armConfig.secondaryCurrentLimit(IntakeConstants.kExtraArmSecondaryCurrentLimit);
		m_armConfig.idleMode(IdleMode.kCoast);
		m_armConfig.inverted(IntakeConstants.kExtraArmInvert);

		m_intakeExtraArm.configure(m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		if (s_theExtraIntakeArm == null) {
			s_theExtraIntakeArm = this;
		} else {
			throw new Error("Intake Extra Arm already instantiated");
		}
	}

	public static IntakeExtraArm getIntakeExtraArm() {
		return s_theExtraIntakeArm;
	}

	/**
	 * Set power of motor driving intake roller wheels.
	 * 
	 * @param speed
	 */
	public static void setMotorPower(double power) {
		s_theExtraIntakeArm.m_intakeExtraArm.set(power);
	}

	public static void stopMotor() {
		s_theExtraIntakeArm.m_intakeExtraArm.stopMotor();
	}

	public static double getPosition() {
		return s_theExtraIntakeArm.m_intakeExtraArm.getEncoder().getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber(
				"Intake Extra Arm/Velocity", s_theExtraIntakeArm.m_intakeExtraArm.getEncoder().getVelocity());
		SmartDashboard.putNumber("Intake Extra Arm/Voltage", s_theExtraIntakeArm.m_intakeExtraArm.getBusVoltage());
		SmartDashboard.putNumber("Intake Extra Arm/Current", s_theExtraIntakeArm.m_intakeExtraArm.getOutputCurrent());
		SmartDashboard.putNumber("Intake Extra Arm/Position", getPosition());
	}
}