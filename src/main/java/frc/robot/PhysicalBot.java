package frc.robot;

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
import frc.robot.Constants.DriveConstants;

public enum PhysicalBot {
	COMP_BOT(() -> new SwerveMotors() {
		private TalonFX m_driveMotor;
		private TalonFX m_steerMotor;

		@Override
		public void configure(int drivePort, int steerPort) {
			m_driveMotor = new TalonFX(drivePort);
			m_steerMotor = new TalonFX(steerPort);
			m_driveMotor.getConfigurator().apply(DriveConstants.kDriveConfig);
			m_steerMotor.getConfigurator().apply(DriveConstants.kSteerConfig);
		}

		@Override
		public DCMotor getSteerGearbox() {
			return DCMotor.getKrakenX60(1);
		}

		@Override
		public DCMotor getDriveGearbox() {
			return DCMotor.getKrakenX60(1);
		}

		@Override
		public void enableCoast() {
			TalonFXConfiguration config = new TalonFXConfiguration();
			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			m_driveMotor.getConfigurator().apply(config);
		}

		@Override
		public void enableBrake() {
			TalonFXConfiguration config = new TalonFXConfiguration();
			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			m_driveMotor.getConfigurator().apply(config);
		}

		@Override
		public double getDriveRotationsInternal() {
			return m_driveMotor.getPosition().getValueAsDouble();
		}

		@Override
		public void setDriveRotationsInternal(double rotations) {
			m_driveMotor.setPosition(rotations);
		}

		@Override
		public double getDriveCurrent() {
			return m_driveMotor.getStatorCurrent().getValueAsDouble();
		}

		@Override
		public double getSteerCurrent() {
			return m_steerMotor.getStatorCurrent().getValueAsDouble();
		}

		@Override
		public double getDriveVoltageInternal() {
			return m_driveMotor.getMotorVoltage().getValueAsDouble();
		}

		@Override
		public double getSteerVoltageInternal() {
			return m_steerMotor.getMotorVoltage().getValueAsDouble();
		}

		@Override
		public void setDriveVoltageInternal(double voltage) {
			m_driveMotor.setVoltage(voltage);
		}

		@Override
		public void setSteerVoltageInternal(double voltage) {
			m_steerMotor.setVoltage(voltage);
		}

	}),
	PRAC_BOT(COMP_BOT::getSwerveMotors),
	DENNIS(() -> new SwerveMotors() {
		private SparkMax m_driveMotor;
		private SparkMax m_steerMotor;

		@Override
		public void configure(int drivePort, int steerPort) {
			m_driveMotor = new SparkMax(drivePort, MotorType.kBrushless);
			m_steerMotor = new SparkMax(steerPort, MotorType.kBrushless);
		}

		@Override
		public DCMotor getSteerGearbox() {
			return DCMotor.getNEO(1);
		}

		@Override
		public DCMotor getDriveGearbox() {
			return DCMotor.getNEO(1);
		}

		@Override
		public void enableCoast() {
			m_driveMotor.configure(
					new SparkMaxConfig()
							.idleMode(IdleMode.kCoast),
					ResetMode.kNoResetSafeParameters,
					PersistMode.kNoPersistParameters);
		}

		@Override
		public void enableBrake() {
			m_driveMotor.configure(
					new SparkMaxConfig()
							.idleMode(IdleMode.kBrake),
					ResetMode.kNoResetSafeParameters,
					PersistMode.kNoPersistParameters);
		}

		@Override
		public double getDriveRotationsInternal() {
			return m_driveMotor.getEncoder().getPosition();
		}

		@Override
		public void setDriveRotationsInternal(double rotations) {
			m_driveMotor.getEncoder().setPosition(rotations);
		}

		@Override
		public double getDriveCurrent() {
			return m_driveMotor.getOutputCurrent();
		}

		@Override
		public double getSteerCurrent() {
			return m_steerMotor.getOutputCurrent();
		}

		@Override
		protected double getDriveVoltageInternal() {
			return m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
		}

		@Override
		protected double getSteerVoltageInternal() {
			return m_steerMotor.getAppliedOutput() * m_steerMotor.getBusVoltage();
		}

		@Override
		protected void setDriveVoltageInternal(double voltage) {
			m_driveMotor.setVoltage(voltage);
		}

		@Override
		protected void setSteerVoltageInternal(double voltage) {
			m_steerMotor.setVoltage(voltage);
		}

	});

	private final Supplier<SwerveMotors> m_motors;

	private PhysicalBot(Supplier<SwerveMotors> motors) {
		m_motors = motors;
	}

	public SwerveMotors getSwerveMotors() {
		return m_motors.get();
	}
}
