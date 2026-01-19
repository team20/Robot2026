package robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import robot.Constants.DriveConstants;
import robot.utilities.SwerveMotors;

/**
 * An enum which represents the robot the code is running on. This will change
 * both the motors used by the drive subsystem and the constants.
 */
public enum PhysicalBot {
	/**
	 * A constant for the competition robot, which uses kraken motors.
	 */
	COMP_BOT(() -> new SwerveMotors() {
		/**
		 * The drive motor uses a talon fx motor controller.
		 */
		private TalonFX m_driveMotor;
		/**
		 * The steer motor uses a talon fx motor controller.
		 */
		private TalonFX m_steerMotor;

		/**
		 * Configures the motor controllers according to the CAN ids as well as the
		 * constants.
		 * 
		 * @param drivePort CAN id of drive motor
		 * @param steerPort CAN id of steer motor
		 */
		@Override
		public void configure(int drivePort, int steerPort) {
			m_driveMotor = new TalonFX(drivePort);
			m_steerMotor = new TalonFX(steerPort);
			m_driveMotor.getConfigurator().apply(DriveConstants.kDriveConfig);
			m_steerMotor.getConfigurator().apply(DriveConstants.kSteerConfig);
		}

		/**
		 * Gets a kraken gearbox for the steer motor.
		 * 
		 * @return gearbox
		 */
		@Override
		public DCMotor getSteerGearbox() {
			return DCMotor.getKrakenX60(1);
		}

		/**
		 * Gets a kraken gearbox for the drive motor.
		 * 
		 * @return gearbox
		 */
		@Override
		public DCMotor getDriveGearbox() {
			return DCMotor.getKrakenX60(1);
		}

		/**
		 * Enables coast mode by changing the motor controller configuration.
		 */
		@Override
		public void enableCoast() {
			TalonFXConfiguration config = new TalonFXConfiguration();
			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			m_driveMotor.getConfigurator().apply(config);
		}

		/**
		 * Enables brake mode by changing the motor controller configuration.
		 */
		@Override
		public void enableBrake() {
			TalonFXConfiguration config = new TalonFXConfiguration();
			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			m_driveMotor.getConfigurator().apply(config);
		}

		/**
		 * Gets the number of rotations made by the drive wheel by polling the relative
		 * encoder.
		 * 
		 * @return distance in rotations
		 */
		@Override
		public double getDriveRotationsInternal() {
			return m_driveMotor.getPosition().getValueAsDouble();
		}

		/**
		 * Sets the number of rotations made by the drive wheel by changing the value of
		 * the relative encoder.
		 * 
		 * @param rotations distance in rotations
		 */
		@Override
		public void setDriveRotationsInternal(double rotations) {
			m_driveMotor.setPosition(rotations);
		}

		/**
		 * Gets the current being used to power the drive motor.
		 * 
		 * @return current in amperes
		 */
		@Override
		public double getDriveCurrent() {
			return m_driveMotor.getStatorCurrent().getValueAsDouble();
		}

		/**
		 * Gets the current being used to power the steer motor.
		 * 
		 * @return current in amperes
		 */
		@Override
		public double getSteerCurrent() {
			return m_steerMotor.getStatorCurrent().getValueAsDouble();
		}

		/**
		 * Gets the voltage being used to power the drive motor.
		 * 
		 * @return voltage in volts
		 */
		@Override
		public double getDriveVoltageInternal() {
			return m_driveMotor.getMotorVoltage().getValueAsDouble();
		}

		/**
		 * Gets the voltage being used to power the steer motor.
		 * 
		 * @return voltage in volts
		 */
		@Override
		public double getSteerVoltageInternal() {
			return m_steerMotor.getMotorVoltage().getValueAsDouble();
		}

		/**
		 * Sets the voltage being used to power the drive motor.
		 * 
		 * @param voltage voltage in volts
		 */
		@Override
		public void setDriveVoltageInternal(double voltage) {
			m_driveMotor.setVoltage(voltage);
		}

		/**
		 * Sets the voltage being used to power the steer motor.
		 * 
		 * @param voltage voltage in volts
		 */
		@Override
		public void setSteerVoltageInternal(double voltage) {
			m_steerMotor.setVoltage(voltage);
		}

	}),
	/**
	 * A constant for the practice robot, which should be the same as the
	 * competition robot, but it might need to be modified at some point.
	 */
	PRAC_BOT(COMP_BOT::getSwerveMotors),
	/**
	 * A constant for dennis, who uses neo motors. Dennis will be forever remembered
	 * as the little bot that could.
	 */
	DENNIS(() -> new SwerveMotors() {
		/**
		 * The drive motor uses a spark max motor controller.
		 */
		private SparkMax m_driveMotor;
		/**
		 * The steer motor uses a spark max motor controller.
		 */
		private SparkMax m_steerMotor;

		/**
		 * Configures the motor controllers according to the CAN ids.
		 * 
		 * @param drivePort CAN id of drive motor
		 * @param steerPort CAN id of steer motor
		 */
		@Override
		public void configure(int drivePort, int steerPort) {
			m_driveMotor = new SparkMax(drivePort, MotorType.kBrushless);
			m_steerMotor = new SparkMax(steerPort, MotorType.kBrushless);
		}

		/**
		 * Gets a neo gearbox for the steer motor.
		 * 
		 * @return gearbox
		 */
		@Override
		public DCMotor getSteerGearbox() {
			return DCMotor.getNEO(1);
		}

		/**
		 * Gets a neo gearbox for the drive motor.
		 * 
		 * @return gearbox
		 */
		@Override
		public DCMotor getDriveGearbox() {
			return DCMotor.getNEO(1);
		}

		/**
		 * Enables coast mode by changing the motor controller configuration.
		 */
		@Override
		public void enableCoast() {
			m_driveMotor.configure(
					new SparkMaxConfig()
							.idleMode(IdleMode.kCoast),
					ResetMode.kNoResetSafeParameters,
					PersistMode.kNoPersistParameters);
		}

		/**
		 * Enables brake mode by changing the motor controller configuration.
		 */
		@Override
		public void enableBrake() {
			m_driveMotor.configure(
					new SparkMaxConfig()
							.idleMode(IdleMode.kBrake),
					ResetMode.kNoResetSafeParameters,
					PersistMode.kNoPersistParameters);
		}

		/**
		 * Gets the number of rotations made by the drive wheel by polling the relative
		 * encoder.
		 * 
		 * @return distance in rotations
		 */
		@Override
		public double getDriveRotationsInternal() {
			return m_driveMotor.getEncoder().getPosition();
		}

		/**
		 * Sets the number of rotations made by the drive wheel by changing the value of
		 * the relative encoder.
		 * 
		 * @param rotations distance in rotations
		 */
		@Override
		public void setDriveRotationsInternal(double rotations) {
			m_driveMotor.getEncoder().setPosition(rotations);
		}

		/**
		 * Gets the current being used to power the drive motor.
		 * 
		 * @return current in amperes
		 */
		@Override
		public double getDriveCurrent() {
			return m_driveMotor.getOutputCurrent();
		}

		/**
		 * Gets the current being used to power the steer motor.
		 * 
		 * @return current in amperes
		 */
		@Override
		public double getSteerCurrent() {
			return m_steerMotor.getOutputCurrent();
		}

		/**
		 * Gets the voltage being used to power the drive motor.
		 * 
		 * @return voltage in volts
		 */
		@Override
		protected double getDriveVoltageInternal() {
			return m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
		}

		/**
		 * Gets the voltage being used to power the steer motor.
		 * 
		 * @return voltage in volts
		 */
		@Override
		protected double getSteerVoltageInternal() {
			return m_steerMotor.getAppliedOutput() * m_steerMotor.getBusVoltage();
		}

		/**
		 * Sets the voltage being used to power the drive motor.
		 * 
		 * @param voltage voltage in volts
		 */
		@Override
		protected void setDriveVoltageInternal(double voltage) {
			m_driveMotor.setVoltage(voltage);
		}

		/**
		 * Sets the voltage being used to power the steer motor.
		 * 
		 * @param voltage voltage in volts
		 */
		@Override
		protected void setSteerVoltageInternal(double voltage) {
			m_steerMotor.setVoltage(voltage);
		}

	});

	/**
	 * Store the lambda in order to create multiple swerve modules
	 */
	private final Supplier<SwerveMotors> m_motors;

	/**
	 * Creates a new constant for each physical robot.
	 * 
	 * @param motors lambda which creates a new {@link SwerveMotors}
	 */
	private PhysicalBot(Supplier<SwerveMotors> motors) {
		m_motors = motors;
	}

	/**
	 * Creates a new set of motors for a swerve module.
	 * 
	 * @return motors
	 */
	public SwerveMotors getSwerveMotors() {
		return m_motors.get();
	}
}
