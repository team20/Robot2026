// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;

/**
 * Contains all the hardware and controllers for a swerve module.
 */
public class SwerveModule {
	private final PIDController m_steerController = new PIDController(kP, kI, kD);
	private final CANcoder m_CANCoder;
	private final TalonFX m_driveMotor;
	private final TalonFX m_steerMotor;
	private final int m_index;
	private final DCMotorSim m_driveMotorModel;
	private final DCMotorSim m_steerMotorModel;
	private Rotation2d m_currentAngle = Rotation2d.kZero;
	private Rotation2d m_goalAngle = Rotation2d.kZero;
	private Distance m_distance = Meters.zero();
	private LinearVelocity m_velocity = MetersPerSecond.zero();

	public SwerveModule(int index, int canId, int drivePort, int steerPort, boolean inverted) {
		m_index = index;
		m_CANCoder = new CANcoder(canId);
		m_driveMotor = new TalonFX(drivePort);
		m_steerMotor = new TalonFX(steerPort);
		TalonFXConfiguration config = DriveConstants.kDriveConfig.clone();
		config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		m_driveMotor.getConfigurator().apply(config);
		m_steerMotor.getConfigurator().apply(DriveConstants.kSteerConfig);
		m_steerController.enableContinuousInput(0, 360);
		if (RobotBase.isSimulation()) {
			m_driveMotorModel = new DCMotorSim(
					LinearSystemId.createDCMotorSystem(kV / (2 * Math.PI), kA / (2 * Math.PI)),
					DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio));
			m_steerMotorModel = new DCMotorSim(
					LinearSystemId.createDCMotorSystem(kV / (2 * Math.PI), kA / (2 * Math.PI)),
					DCMotor.getKrakenX60(1));
		} else {
			m_driveMotorModel = null;
			m_steerMotorModel = null;
		}
	}

	public int getIndex() {
		return m_index;
	}

	public NeutralModeValue setNeutralMode(NeutralModeValue neutralMode) {
		var config = new TalonFXConfiguration();
		config.MotorOutput.NeutralMode = neutralMode;
		m_driveMotor.getConfigurator().apply(config);
		return neutralMode;
	}

	/**
	 * Returns drive encoder distance in meters traveled.
	 * 
	 * @return The position in meters.
	 */
	public Distance getDriveEncoderPosition() {
		return m_distance;
	}

	/**
	 * Returns drive encoder velocity.
	 * 
	 * @return The velocity in meters/s.
	 */
	public LinearVelocity getDriveEncoderVelocity() {
		return m_velocity;
	}

	/**
	 * Resets drive encoder to zero.
	 */
	public void resetDriveEncoder() {
		m_driveMotor.setPosition(0);
	}

	/**
	 * Returns the module angle in degrees.
	 * 
	 * @return The module angle
	 */
	public Rotation2d getModuleAngle() {
		return m_currentAngle;
	}

	/**
	 * Returns the module position.
	 * 
	 * @return The module position
	 */
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(getDriveEncoderPosition(), getModuleAngle());
	}

	/**
	 * Gets the module speed and angle.
	 * 
	 * @return The module state
	 */
	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(getDriveEncoderVelocity(), getModuleAngle());
	}

	/**
	 * Sets the drive motor speeds and module angle.
	 * 
	 * @param state The module state. Note that the speedMetersPerSecond field has
	 *        been repurposed to contain power between -1 and 1, not velocity.
	 */
	public void setModuleState(SwerveModuleState state) {
		state.cosineScale(m_currentAngle);
		state.optimize(m_currentAngle);
		setDrivePower(state.speedMetersPerSecond);
		setSteerAngle(state.angle);
	}

	/**
	 * Sets the drive motor power level
	 * 
	 * @param power power level
	 * @return actual output power level
	 */
	public void setDrivePower(double power) {
		m_driveMotor.set(ABBA.preventBrownout(power) * kMaxThrottle);
	}

	/**
	 * Sets the steer motor power level
	 * 
	 * @param power power level
	 * @return actual output power level
	 */
	public void setSteerAngle(Rotation2d angle) {
		m_goalAngle = angle;
	}

	public void periodic() {
		// Update drive motor
		m_distance = Meters.of(m_driveMotor.getPosition().getValueAsDouble() * kMetersPerMotorRotation);
		m_velocity = MetersPerSecond.of(m_driveMotor.getVelocity().getValueAsDouble() * kMetersPerMotorRotation);

		// Update steer motor
		m_currentAngle = new Rotation2d(m_CANCoder.getAbsolutePosition().getValue());
		double power = m_steerController.calculate(m_currentAngle.getDegrees(), m_goalAngle.getDegrees());
		m_steerMotor.set(ABBA.preventBrownout(power));

		if (RobotBase.isSimulation()) {
			var driveMotorState = m_driveMotor.getSimState();
			m_driveMotorModel.setInputVoltage(driveMotorState.getMotorVoltage());
			m_driveMotorModel.update(0.02);
			driveMotorState.setRotorVelocity(m_driveMotorModel.getAngularVelocityRPM() / 60);
			driveMotorState.setRawRotorPosition(m_driveMotorModel.getAngularPositionRotations());

			var encoderSimState = m_CANCoder.getSimState();
			var steerMotorState = m_steerMotor.getSimState();
			m_steerMotorModel.setInputVoltage(steerMotorState.getMotorVoltage());
			m_steerMotorModel.update(0.02);
			encoderSimState.setRawPosition(m_steerMotorModel.getAngularPositionRotations() / kSteerGearRatio);
			encoderSimState.setVelocity(m_steerMotorModel.getAngularVelocityRPM() / 60 / kSteerGearRatio);
		}
	}
}