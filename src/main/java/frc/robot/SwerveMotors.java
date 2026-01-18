package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

public abstract class SwerveMotors {
	private double m_driveVoltage;
	private double m_steerVoltage;
	private double m_driveRotations;

	public abstract void configure(int drivePort, int steerPort);

	public abstract DCMotor getSteerGearbox();

	public abstract DCMotor getDriveGearbox();

	public abstract void enableCoast();

	public abstract void enableBrake();

	public abstract double getDriveRotationsInternal();

	public abstract void setDriveRotationsInternal(double rotations);

	public abstract double getDriveCurrent();

	public abstract double getSteerCurrent();

	protected abstract double getDriveVoltageInternal();

	protected abstract double getSteerVoltageInternal();

	protected abstract void setDriveVoltageInternal(double voltage);

	protected abstract void setSteerVoltageInternal(double voltage);

	public double getDriveVoltage() {
		return switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> getDriveVoltageInternal();
			case kSimulation -> m_driveVoltage;
		};
	}

	public double getSteerVoltage() {
		return switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> getSteerVoltageInternal();
			case kSimulation -> m_steerVoltage;
		};
	}

	public void setDriveVoltage(double voltage) {
		switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> setDriveVoltageInternal(voltage);
			case kSimulation -> m_driveVoltage = voltage;
		}
	}

	public void setSteerVoltage(double voltage) {
		switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> setSteerVoltageInternal(voltage);
			case kSimulation -> m_steerVoltage = voltage;
		}
	}

	public double getDriveRotations() {
		return switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> getDriveRotationsInternal();
			case kSimulation -> m_driveRotations;
		};
	};

	public void setDriveRotations(double rotations) {
		switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> setDriveRotationsInternal(rotations);
			case kSimulation -> m_driveRotations = rotations;
		}
	};

	public void resetDriveRotations() {
		setDriveRotations(0);
	};
}
