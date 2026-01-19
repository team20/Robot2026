// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import robot.Constants.DriveConstants;
import robot.utilities.BotConstants;
import robot.utilities.SwerveMotors;

/**
 * Contains all the hardware and controllers for a swerve module.
 */
public class SwerveModule {
	private final PIDController m_steerController = new PIDController(DriveConstants.kP, DriveConstants.kI,
			DriveConstants.kD);
	private final CANcoder m_CANCoder;
	private final SwerveMotors m_motors;

	// private final SparkFlexSim m_steerMotorSim;
	private final DCMotorSim m_driveMotorModel;
	private final DCMotorSim m_steerMotorModel;
	private final BotConstants m_constants;

	public SwerveModule(PhysicalBot bot, int canId, int drivePort, int steerPort) {
		m_constants = DriveConstants.kBots.get(bot);
		m_CANCoder = new CANcoder(canId);
		m_motors = bot.getSwerveMotors();
		m_motors.configure(drivePort, steerPort);
		m_steerController.enableContinuousInput(0, 360);
		if (RobotBase.isSimulation()) {
			m_driveMotorModel = new DCMotorSim(
					LinearSystemId
							.createDCMotorSystem(DriveConstants.kV / (2 * Math.PI), DriveConstants.kA / (2 * Math.PI)),
					m_motors.getDriveGearbox().withReduction(m_constants.kDriveGearRatio));
			m_steerMotorModel = new DCMotorSim(
					LinearSystemId
							.createDCMotorSystem(DriveConstants.kV / (2 * Math.PI), DriveConstants.kA / (2 * Math.PI)),
					m_motors.getSteerGearbox());
		} else {
			m_driveMotorModel = null;
			m_steerMotorModel = null;
		}
	}

	public void setNeutralMode(NeutralModeValue neutralMode) {
		switch (neutralMode) {
			case Brake -> m_motors.enableBrake();
			case Coast -> m_motors.enableCoast();
		}
	}

	/**
	 * Returns drive encoder distance in meters traveled.
	 * 
	 * @return The position in meters.
	 */
	public double getDriveEncoderPosition() {
		return m_motors.getDriveRotations() * m_constants.getMetersPerMotorRotation();
	}

	/**
	 * Returns the current of the steer motor
	 * 
	 * @return The current in amps
	 */
	public double getSteerCurrent() {
		return m_motors.getSteerCurrent();
	}

	/**
	 * Returns the current of the drive motor
	 * 
	 * @return The current in amps
	 */
	public double getDriveCurrent() {
		return m_motors.getDriveCurrent();
	}

	/**
	 * Resets drive encoder to zero.
	 */
	public void resetDriveEncoder() {
		m_motors.resetDriveRotations();
	}

	/**
	 * Gets the current drive motor voltage.
	 * 
	 * @return The motor speed in voltage
	 */
	public double getDriveVoltage() {
		return m_motors.getDriveVoltage();
	}

	/**
	 * Gets the current drive motor voltage.
	 * 
	 * @return The motor speed in voltage
	 */
	public double getSteerVoltage() {
		return m_motors.getSteerVoltage();
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
	 *        been repurposed to contain volts, not velocity.
	 */
	public void setModuleState(SwerveModuleState state) {
		m_motors.setDriveVoltage(state.speedMetersPerSecond);
		double turnPower = m_steerController.calculate(getModuleAngle(), state.angle.getDegrees());
		m_motors.setSteerVoltage(turnPower);
		updateSim();
	}

	private void updateSim() {
		if (RobotBase.isSimulation()) {
			m_driveMotorModel.setInputVoltage(m_motors.getDriveVoltage());
			m_driveMotorModel.update(0.02);
			m_motors.setDriveRotations(m_driveMotorModel.getAngularPositionRotations());

			var encoderSimState = m_CANCoder.getSimState();
			m_steerMotorModel.setInputVoltage(m_motors.getSteerVoltage());
			m_steerMotorModel.update(0.02);
			encoderSimState
					.setRawPosition(m_steerMotorModel.getAngularPositionRotations() / m_constants.kSteerGearRatio);
			encoderSimState.setVelocity(m_steerMotorModel.getAngularVelocityRPM() / 60 / m_constants.kSteerGearRatio);
		}
	}
}
