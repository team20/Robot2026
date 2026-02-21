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

	public AngularPositionSubsystem(int id, String name,
			double kP, double kI,
			int currentLimit, int smartCurrentLimit,
			double minAngle, double maxAngle,
			double maxDutyCycle, boolean inverted) {
		m_name = name;
		m_minAngle = minAngle;
		m_maxAngle = maxAngle;
		m_maxDutyCycle = maxDutyCycle;
		m_motor = new SparkMax(id, MotorType.kBrushless);
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake);
		config.absoluteEncoder.positionConversionFactor(360);
		config.closedLoop.pid(kP, kI, 0);
		config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
		config.closedLoop.positionWrappingEnabled(false);
		config.smartCurrentLimit(smartCurrentLimit);
		config.secondaryCurrentLimit(currentLimit);
		config.inverted(inverted);
		m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_encoder = m_motor.getAbsoluteEncoder();
		m_controller = m_motor.getClosedLoopController();
	}

	public void setAngle(double angle) {
		if (angle < m_minAngle || angle > m_maxAngle) {
			return;
		}
		m_dutyCycle = 0;
		m_controller.setSetpoint(angle, ControlType.kPosition);
	}

	public double getPosition() {
		double position = m_encoder.getPosition();
		/*
		 * if (position < m_minAngle || position > m_disallowedMidpoint) {
		 * return m_minAngle;
		 * } else if (position > m_maxAngle && position < m_disallowedMidpoint) {
		 * return m_maxAngle;
		 * }
		 */
		return position;
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

	private double minDifference(double a, double b) {
		double difference = Math.abs(a - b);
		return Math.min(difference, 360 - difference);
	}

	private boolean limited() {
		double position = getPosition();
		if (position < m_maxAngle && position > m_minAngle) {
			return false;
		}
		double distanceToMax = minDifference(position, m_maxAngle);
		double distanceToMin = minDifference(position, m_minAngle);
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
		}
	}
}
