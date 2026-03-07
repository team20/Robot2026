// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

	// private final SparkFlexSim m_steerMotorSim;
	private final DCMotorSim m_driveMotorModel;
	private final DCMotorSim m_steerMotorModel;

	public SwerveModule(int index, int canId, int drivePort, int steerPort, boolean Inverted) {
		m_index = index;
		m_CANCoder = new CANcoder(canId);
		m_driveMotor = new TalonFX(drivePort);
		m_steerMotor = new TalonFX(steerPort);
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
	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(getDriveVoltage(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Sets the drive motor speeds and module angle.
	 * 
	 * @param state The module state. Note that the speedMetersPerSecond field has
	 *        been repurposed to contain power between -1 and 1, not velocity.
	 */
	public SwerveModuleState setModuleState(SwerveModuleState state) {
		m_driveMotor.set(ABBA.preventBrownout(state.speedMetersPerSecond));
		double turnPower = m_steerController.calculate(getModuleAngle(), state.angle.getDegrees());
		m_steerMotor.set(ABBA.preventBrownout(turnPower));
		updateSim();
		return state;
	}

	private void updateSim() {
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