package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.TurretConstants;

/**
 * A subsystem which has the ability to change both yaw and pitch in order to
 * adjust the direction fuel will exit the shooter.
 */
public class Turret extends SubsystemBase {
	/**
	 * The yaw motor allows the mechanism to pivot left and right.
	 */
	private final SparkMax m_motor;
	/**
	 * The through bore encoder on the yaw axis allows us to check the yaw angle.
	 */
	private final SparkAbsoluteEncoder m_encoder;

	private final SparkClosedLoopController m_controller;
	/**
	 * The through bore encoder on the pitch axis allows us to check the pitch
	 * angle.
	 * /**
	 * The yaw PID controller is used to aim the turret in a specific yaw direction
	 * in a closed feedback loop.
	 */
	// private final PIDController m_pid;

	/**
	 * The pitch PID controller is used to aim the turret in a specific pitch
	 * direction in a closed feedback loop.
	 */

	/**
	 * Creates a new subsystem using the motor id's from the turret constants, the
	 * corresponding pid constants from the turret constants, and a proper name.
	 */
	public Turret() {
		m_motor = new SparkMax(2, MotorType.kBrushless);
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kCoast);
		config.absoluteEncoder.positionConversionFactor(360 / TurretConstants.kGearRatio);
		config.closedLoop.pid(TurretConstants.kP, 0, 0);
		config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
		m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_encoder = m_motor.getAbsoluteEncoder();
		m_controller = m_motor.getClosedLoopController();
	}

	public void setAngle(double angle) {
		m_controller.setSetpoint(angle, ControlType.kPosition);
	}

	public double getPosition() {
		return m_encoder.getPosition();
	}

	public void runMotorAtDutyCycle(double dutyCycle) {
		double sign = Math.signum(dutyCycle);
		dutyCycle = Math.min(TurretConstants.kMaxDutyCycle, Math.abs(dutyCycle));

		m_motor.set(dutyCycle * sign);
	}
}
