// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CANConstants;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.framework.RobotBase;
import org.wpilib.simulation.DCMotorSim;
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

	// private final SparkFlexSim m_steerMotorSim;
	private final DCMotorSim m_driveMotorModel;
	private final DCMotorSim m_steerMotorModel;

	public SwerveModule(int index, int canId, int drivePort, int steerPort, boolean Inverted) {
		m_index = index;
		m_CANCoder = new CANcoder(canId, CANConstants.kDriveBus);
		m_driveMotor = new TalonFX(drivePort, CANConstants.kDriveBus);
		m_steerMotor = new TalonFX(steerPort, CANConstants.kDriveBus);
		// m_steerMotorSim = new SparkFlexSim(m_steerMotor, DCMotor.getNEO(1));
		TalonFXConfiguration config = DriveConstants.kDriveConfig.clone();
		config.MotorOutput.Inverted = Inverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		m_driveMotor.getConfigurator().apply(config);
		// Helps with encoder precision (not set in stone)
		// config.encoder.uvwAverageDepth(kEncoderDepth).uvwMeasurementPeriod(kEncoderMeasurementPeriod);
		m_steerMotor.getConfigurator().apply(DriveConstants.kSteerConfig);
		m_steerController.enableContinuousInput(0, 360);
		if (RobotBase.isSimulation()) {
			m_driveMotorModel = new DCMotorSim(
					Models.singleJointedArmFromSysId(kV / (2 * Math.PI), kA / (2 * Math.PI)),
					DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio));
			m_steerMotorModel = new DCMotorSim(
					Models.singleJointedArmFromSysId(kV / (2 * Math.PI), kA / (2 * Math.PI)),
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
	public double getDriveEncoderPosition() {
		return m_driveMotor.getPosition().getValueAsDouble() * kMetersPerMotorRotation;
	}

	/**
	 * Returns the current of the steer motor
	 * 
	 * @return The current in amps
	 */
	public double getSteerCurrent() {
		return m_steerMotor.getStatorCurrent().getValueAsDouble();
	}

	/**
	 * Returns the current of the drive motor
	 * 
	 * @return The current in amps
	 */
	public double getDriveCurrent() {
		return m_driveMotor.getStatorCurrent().getValueAsDouble();
	}

	/**
	 * Resets drive encoder to zero.
	 */
	public boolean resetDriveEncoder() {
		m_driveMotor.setPosition(0);
		return getDriveEncoderPosition() == 0;
	}

	/**
	 * Gets the current drive motor voltage.
	 * 
	 * @return The motor speed in voltage
	 */
	public double getDriveVoltage() {
		return m_driveMotor.getMotorVoltage().getValueAsDouble();
	}

	/**
	 * Gets the current drive motor temperature.
	 * 
	 * @return The temperature in degrees Celsius
	 */
	public double getDriveTemperature() {
		return m_driveMotor.getDeviceTemp().getValueAsDouble();
	}

	/**
	 * Returns the module angle in degrees.
	 * 
	 * @return The module angle
	 */
	public double getModuleAngle() {
		return m_CANCoder.getAbsolutePosition().getValueAsDouble() * 360;
	}

	/**
	 * Returns the module position.
	 * 
	 * @return The module position
	 */
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(getDriveEncoderPosition(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Gets the module speed and angle.
	 * 
	 * @return The module state
	 */
	public SwerveModuleVelocity getModuleState() {
		return new SwerveModuleVelocity(getDriveVoltage(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Sets the drive motor speeds and module angle.
	 * 
	 * @param state The module state. Note that the velocity field has
	 *        been repurposed to contain power between -1 and 1, not velocity.
	 */
	public SwerveModuleVelocity setModuleState(SwerveModuleVelocity state) {
		m_driveMotor.setThrottle(ABBA.preventBrownout(state.velocity));
		double turnPower = m_steerController.calculate(getModuleAngle(), state.angle.getDegrees());
		m_steerMotor.setThrottle(ABBA.preventBrownout(turnPower));
		Telemetry.log(
				String.format("Swerve Supply Currents (A)/%d", m_index),
				m_driveMotor.getSupplyCurrent().getValueAsDouble());
		Telemetry.log(
				String.format("Swerve Torque Currents (A)/%d", m_index),
				m_driveMotor.getTorqueCurrent().getValueAsDouble());
		Telemetry.log(String.format("Swerve Angles (deg)/%d", m_index), getModuleAngle());
		Telemetry.log(
				String.format("Swerve Velocities (m\\s)/%d", m_index),
				m_driveMotor.getVelocity().getValueAsDouble() * DriveConstants.kMetersPerMotorRotation);
		updateSim();
		return state;
	}

	private void updateSim() {
		if (RobotBase.isSimulation()) {
			var driveMotorState = m_driveMotor.getSimState();
			m_driveMotorModel.setInputVoltage(driveMotorState.getMotorVoltage());
			m_driveMotorModel.update(0.02);
			driveMotorState.setRotorVelocity(m_driveMotorModel.getAngularVelocity() / (2 * Math.PI)); // rad/s -> rot/s
			driveMotorState.setRawRotorPosition(m_driveMotorModel.getAngularPosition() / (2 * Math.PI)); // rad -> rot

			var encoderSimState = m_CANCoder.getSimState();
			var steerMotorState = m_steerMotor.getSimState();
			m_steerMotorModel.setInputVoltage(steerMotorState.getMotorVoltage());
			m_steerMotorModel.update(0.02);
			encoderSimState.setRawPosition(m_steerMotorModel.getAngularPosition() / (2 * Math.PI) / kSteerGearRatio); // rad -> rot
			encoderSimState.setVelocity(m_steerMotorModel.getAngularVelocity() / (2 * Math.PI) / kSteerGearRatio); // rad/s -> rot/s
		}
	}
}