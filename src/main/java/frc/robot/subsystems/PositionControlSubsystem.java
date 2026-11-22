package frc.robot.subsystems;

import frc.robot.Constants.CANConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import frc.robot.commands.compat.SubsystemBase;

public class PositionControlSubsystem extends SubsystemBase {
	private final SparkBase m_motor;
	private final double m_maxPower;
	// Mechanism units per motor-encoder rotation. REVLib 2027 removed encoder conversion factors,
	// so the SPARK always works in rotations; positions are converted here instead.
	// Subclasses must also scale their on-SPARK kP/kI by this factor to keep the same tuning.
	private final double m_unitsPerRotation;

	public PositionControlSubsystem(int port, boolean sparkMax, double maxPower) {
		this(port, sparkMax, maxPower, 1.0);
	}

	public PositionControlSubsystem(int port, boolean sparkMax, double maxPower, double unitsPerRotation) {
		m_maxPower = maxPower;
		m_unitsPerRotation = unitsPerRotation;
		if (sparkMax) {
			m_motor = new SparkMax(CANConstants.kMechanismPort, port, MotorType.kBrushless);

		} else {
			m_motor = new SparkFlex(CANConstants.kMechanismPort, port, MotorType.kBrushless);

		}
	}

	public void setMotorPosition(double position) {
		m_motor.getClosedLoopController().setSetpoint(position / m_unitsPerRotation, ControlType.kPosition);
	}

	public void setMotorPower(double power) {
		// Clamp power between (-m_maxPower, m_maxPower)
		if (Math.abs(power) > m_maxPower) {
			power = Math.signum(power) * m_maxPower;
		}
		m_motor.setThrottle(power);
	}

	public double getMotorRotations() {
		return m_motor.getEncoder().getPosition().get() * m_unitsPerRotation;
	}

	public void stopMotor() {
		m_motor.stopMotor();
	}

	public void resetMotorEncoder() {
		m_motor.getEncoder().setPosition(0);
	}

	public void resetMotorEncoderOffset(double offset) {
		m_motor.getEncoder().setPosition(offset / m_unitsPerRotation);
	}

	public void configureMotor(SparkMaxConfig config) {
		m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	// Telemetry table named after the concrete subclass (same path as the old SmartDashboard keys)
	private final TelemetryTable m_telemetry = Telemetry.getTable(getClass().getName());

	@Override
	public void periodic() {
		m_telemetry.log("Motor Position", getMotorRotations());
		m_telemetry.log("Motor Speed", m_motor.getThrottle());
		m_telemetry.log(
				"Error",
				(m_motor.getEncoder().getPosition().get() - m_motor.getClosedLoopController().getSetpoint().get())
						* m_unitsPerRotation);
	}
}
