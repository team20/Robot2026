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
import frc.robot.Constants.Subsystems.HoodConstants;

public class Hood extends SubsystemBase {
	private static Hood s_theHood;
	private final SparkMax m_motor;

	private final SparkAbsoluteEncoder m_encoder;

	private final SparkClosedLoopController m_controller;

	public Hood() {
		m_motor = new SparkMax(2, MotorType.kBrushless);
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake);
		config.absoluteEncoder.positionConversionFactor(360);
		config.closedLoop.pid(HoodConstants.kP, 0, 0);
		config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
		m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_encoder = m_motor.getAbsoluteEncoder();
		m_controller = m_motor.getClosedLoopController();

		if (s_theHood == null) {
			s_theHood = this;
		} else {
			throw new Error("Hood already instantiated");
		}
	}

	public static Hood getHood() {
		return s_theHood;
	}

	public static void setAngle(double angle) {
		s_theHood.m_controller.setSetpoint(angle, ControlType.kPosition);
	}

	public static double getPosition() {
		return s_theHood.m_encoder.getPosition();
	}

	public static void runAtDutyCycle(double dutyCycle) {
		double sign = Math.signum(dutyCycle);
		dutyCycle = Math.min(HoodConstants.kMaxDutyCycle, Math.abs(dutyCycle));

		s_theHood.m_motor.set(dutyCycle * sign);
	}

	public static void stop() {
		s_theHood.m_motor.stopMotor();
	}

	@Override
	public void periodic() {
		if (Constants.kLogging) {
			SmartDashboard.putNumber("Hood/Position", getPosition());
		}
	}
}