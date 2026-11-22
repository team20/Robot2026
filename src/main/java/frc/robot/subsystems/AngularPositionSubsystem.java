package frc.robot.subsystems;

import frc.robot.Constants.CANConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import frc.robot.commands.compat.SubsystemBase;
import frc.robot.AngleUtility;
import frc.robot.Constants;

/**
 * An angular position subsystem is a motor controlled by a spark max attached
 * to an absolute encoder. This class makes it easy to move to a specific
 * angle/position and also perform relative movements within set boundaries.
 */
public class AngularPositionSubsystem extends SubsystemBase {
	private double m_dutyCycle;
	private final SparkMax m_motor;
	private final double m_minAngle;
	private final double m_maxAngle;
	private final String m_name;
	// Telemetry table under the subsystem name (same path as the old SmartDashboard keys)
	private final TelemetryTable m_telemetry;
	private final double m_maxDutyCycle;
	// Degrees per absolute-encoder rotation. REVLib 2027 removed encoder conversion factors, so the
	// SPARK works in rotations; everything outside the SPARK stays in degrees and is converted here.
	private final double m_degreesPerRotation;
	private final SparkAbsoluteEncoder m_encoder;
	private final SparkClosedLoopController m_controller;
	private double m_setpoint;
	private double m_tolerance;

	public AngularPositionSubsystem(int id, String name,
			double kP, double kI, double kS,
			int currentLimit, int smartCurrentLimit,
			double minAngle, double maxAngle,
			double conversionFactor,
			double maxDutyCycle, boolean motorInverted, boolean encoderInverted, double tolerance) {
		m_name = name;
		m_telemetry = Telemetry.getTable(name);
		m_minAngle = minAngle;
		m_maxAngle = maxAngle;
		m_maxDutyCycle = maxDutyCycle;
		m_motor = new SparkMax(CANConstants.kMechanismPort, id, MotorType.kBrushless);
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake);
		m_degreesPerRotation = 360 * conversionFactor;
		config.absoluteEncoder.inverted(encoderInverted);
		// The SPARK now sees error in rotations, not degrees; scale kP/kI to keep the 2026 tuning
		config.closedLoop.pid(kP * m_degreesPerRotation, kI * m_degreesPerRotation, 0);
		config.closedLoop.feedForward.kS(kS);
		config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
		config.closedLoop.positionWrappingEnabled(false);
		config.smartCurrentLimit(smartCurrentLimit);
		config.secondaryCurrentLimit(currentLimit);
		config.inverted(motorInverted);
		m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_encoder = m_motor.getAbsoluteEncoder();
		m_controller = m_motor.getClosedLoopController();
		m_tolerance = tolerance;
		Telemetry.log("Aim Distance", 0.0); // double, to match the type AimCommands logs
	}

	public void moveToPosition(double position) {
		setSetpoint(position);
		m_controller.setSetpoint(m_setpoint / m_degreesPerRotation, ControlType.kPosition);
	}

	public void setSetpoint(double setpointPosition) {
		if (setpointPosition < m_minAngle) {
			setpointPosition = m_minAngle;
		} else if (setpointPosition > m_maxAngle) {
			setpointPosition = m_maxAngle;
		}

		m_dutyCycle = 0;
		m_setpoint = setpointPosition;
	}

	public double getSetpoint() {
		return m_setpoint;
	}

	public boolean isSettled(double tolerance) {
		return Math.abs(getPosition() - m_setpoint) <= tolerance;
	}

	public double getPosition() {
		// TODO(2027 migration): decide how limited() should handle a stale encoder reading.
		// Behavior here is identical to 2026: .get() returns the last known value, even if the
		// SPARK has stopped sending status frames (loose CAN, bus overload, SPARK reboot). The
		// soft limit in limited() can then act on an old angle and let the motor drive past
		// m_minAngle/m_maxAngle. 2027 REVLib exposes Signal.isValid() to detect this. Options:
		// A. Keep as-is: use the last known value (current behavior, same as 2026).
		// B. Fail safe: treat an invalid reading as limited and stop the motor. Never bypasses
		//    the limit, but a brief CAN glitch freezes the mechanism mid-match.
		// C. Fail safe after a grace period: tolerate a few invalid loops (~100-200 ms), then
		//    stop and raise an alert. Recommended once real bus behavior on SystemCore is known.
		// Note: isValid() only covers CAN delivery; an unplugged absolute encoder may still
		// report "valid" but wrong values.
		return m_encoder.getPosition().get() * m_degreesPerRotation;
	}

	public void runAtDutyCycle(double dutyCycle) {
		double sign = Math.signum(dutyCycle);
		dutyCycle = Math.min(m_maxDutyCycle, Math.abs(dutyCycle));
		m_dutyCycle = dutyCycle * sign;
		m_motor.setThrottle(limited() ? 0 : m_dutyCycle);
	}

	public void stop() {
		m_motor.stopMotor();
	}

	private boolean limited() {
		if (Constants.kLogging) {
			Telemetry.log(m_name + " current draw (A)", m_motor.getOutputCurrent().get());
		}
		double position = getPosition();
		if (position < m_maxAngle && position > m_minAngle) {
			return false;
		}
		double distanceToMax = AngleUtility.minDifference(position, m_maxAngle);
		double distanceToMin = AngleUtility.minDifference(position, m_minAngle);
		if (distanceToMin < distanceToMax) {
			return m_dutyCycle < 0; // We overshot the minimum, no negative power
		} else {
			return m_dutyCycle > 0; // We overshot the maximum, no positive power
		} // Ensure that you cannot overshoot even more after overshooting has already
			// ocurred.
	}

	@Override
	public void periodic() {
		if (limited()) {
			stop();
		}
		if (Constants.kLogging) {
			m_telemetry.log("Position", getPosition());
			m_telemetry.log("Setpoint", m_setpoint);
			m_telemetry.log("Setpoint Upper Threshold", m_setpoint + m_tolerance);
			m_telemetry.log("Setpoint Lower Threshold", m_setpoint - m_tolerance);
		}
	}
}
