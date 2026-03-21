package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
	private final double m_maxDutyCycle;
	private final SparkAbsoluteEncoder m_encoder;
	private final SparkClosedLoopController m_controller;
	private double m_setpoint;
	private double m_tolerance;

	public AngularPositionSubsystem(int id, String name,
			double kP, double kI, double kS,
			int currentLimit, int smartCurrentLimit,
			double minAngle, double maxAngle,
			double maxDutyCycle, boolean motorInverted, boolean encoderInverted, double tolerance) {
		m_name = name;
		m_minAngle = minAngle;
		m_maxAngle = maxAngle;
		m_maxDutyCycle = maxDutyCycle;
		m_motor = new SparkMax(id, MotorType.kBrushless);
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake);
		config.absoluteEncoder.positionConversionFactor(360);
		config.absoluteEncoder.inverted(encoderInverted);
		config.closedLoop.pid(kP, kI, 0);
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
		SmartDashboard.putNumber("Aim Distance", 0);
	}

	public void moveToAngle(double angle) {
		setSetpoint(angle);
		m_controller.setSetpoint(m_setpoint, ControlType.kPosition);
	}

	public void setSetpoint(double setpoint) {
		if (setpoint < m_minAngle) {
			setpoint = m_minAngle;
		} else if (setpoint > m_maxAngle) {
			setpoint = m_maxAngle;
		}

		m_dutyCycle = 0;
		m_setpoint = setpoint;
	}

	public double getSetpoint() {
		return m_setpoint;
	}

	public boolean isSettled(double tolerance) {
		return Math.abs(getPosition() - m_setpoint) <= tolerance;
	}

	public double getPosition() {
		return m_encoder.getPosition();
	}

	public void runAtDutyCycle(double dutyCycle) {
		double sign = Math.signum(dutyCycle);
		dutyCycle = Math.min(m_maxDutyCycle, Math.abs(dutyCycle));
		m_dutyCycle = dutyCycle * sign;
		m_motor.set(limited() ? 0 : m_dutyCycle);
	}

	public void stop() {
		m_motor.stopMotor();
	}

	private boolean limited() {
		if (Constants.kLogging) {
			SmartDashboard.putNumber(String.format("%s current draw (A)", m_name), m_motor.getOutputCurrent());
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
			SmartDashboard.putNumber(String.format("%s/Position", m_name), getPosition());
			SmartDashboard.putNumber(String.format("%s/Setpoint", m_name), m_setpoint);
			SmartDashboard.putNumber(String.format("%s/Setpoint Upper Threshold", m_name), m_setpoint + m_tolerance);
			SmartDashboard.putNumber(String.format("%s/Setpoint Lower Threshold", m_name), m_setpoint - m_tolerance);
		}
	}
}
