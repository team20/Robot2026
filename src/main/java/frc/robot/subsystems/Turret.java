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
import frc.robot.Constants.Subsystems.TurretConstants;

public class Turret extends SubsystemBase {
	private static Turret s_theTurret;
	private final SparkMax m_motor;

	private final SparkAbsoluteEncoder m_encoder;

	private final SparkClosedLoopController m_controller;

	public Turret() {
		m_motor = new SparkMax(2, MotorType.kBrushless);
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake);
		config.absoluteEncoder.positionConversionFactor(360);
		config.closedLoop.pid(TurretConstants.kP, TurretConstants.kI, 0);
		config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
		config.closedLoop.positionWrappingEnabled(false);
		config.inverted(true);
		// config.closedLoop.
		config.closedLoop.positionWrappingInputRange(0, 360);
		config.smartCurrentLimit(20);
		config.secondaryCurrentLimit(25);
		m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_encoder = m_motor.getAbsoluteEncoder();
		m_controller = m_motor.getClosedLoopController();

		if (s_theTurret == null) {
			s_theTurret = this;
		} else {
			throw new Error("Turret already instantiated");
		}
	}

	public static Turret getTurret() {
		return s_theTurret;
	}

	public static void setAngle(double angle) {
		s_theTurret.m_controller.setSetpoint(angle, ControlType.kPosition);
	}

	public static double getPosition() {
		return s_theTurret.m_encoder.getPosition();
	}

	public static void runAtDutyCycle(double dutyCycle) {
		if (dutyCycle > 0) {
			dutyCycle *= Math.min((220 - Turret.getPosition()) / 25, 1);
		} else {
			dutyCycle *= Math.min(Turret.getPosition() / 25, 1);
		}
		double sign = Math.signum(dutyCycle);
		dutyCycle = Math.min(TurretConstants.kMaxDutyCycle, Math.abs(dutyCycle));
		s_theTurret.m_motor.set(dutyCycle * sign);
	}

	public static void stop() {
		s_theTurret.m_motor.stopMotor();
	}

	@Override
	public void periodic() {
		if (Constants.kLogging) {
			SmartDashboard.putNumber("Turret/Position", getPosition());
		}
	}
}