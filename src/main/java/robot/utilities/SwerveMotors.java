package robot.utilities;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * A class which allows having multiple different types of swerve drive
 * hardware. This makes it easy to switch between different robots.
 */
public abstract class SwerveMotors {
	/**
	 * The drive voltage is stored for simulation purposes.
	 */
	private double m_driveVoltage;
	/**
	 * The steer voltage is stored for simulation purposes.
	 */
	private double m_steerVoltage;
	/**
	 * The number of drive rotations is stored for simulation purposes.
	 */
	private double m_driveRotations;

	/**
	 * Configures the underlying hardware with CAN ids and anything else needed.
	 * 
	 * @param drivePort CAN id of drive motor
	 * @param steerPort CAN id of steer motor
	 */
	public abstract void configure(int drivePort, int steerPort);

	/**
	 * Gets a gearbox in order to simulate the steer motor.
	 * 
	 * @return gearbox
	 */
	public abstract DCMotor getSteerGearbox();

	/**
	 * Gets a gearbox in order to simulate the drive motor.
	 * 
	 * @return gearbox
	 */
	public abstract DCMotor getDriveGearbox();

	/**
	 * Enables coast mode on the underlying drive motor controller.
	 */
	public abstract void enableCoast();

	/**
	 * Enables brake mode on the underlying drive motor controller.
	 */
	public abstract void enableBrake();

	/**
	 * Gets the number of rotations made by the drive wheel.
	 * 
	 * @return distance in rotations
	 */
	public abstract double getDriveRotationsInternal();

	/**
	 * Sets the number of rotations made by the drive wheel
	 * 
	 * @param rotations distance in rotations
	 */
	public abstract void setDriveRotationsInternal(double rotations);

	/**
	 * Gets the current being used to power the drive motor.
	 * 
	 * @return current in amperes
	 */
	public abstract double getDriveCurrent();

	/**
	 * Gets the current being used to power the steer motor.
	 * 
	 * @return current in amperes
	 */
	public abstract double getSteerCurrent();

	/**
	 * Gets the voltage being used to power the drive motor.
	 * 
	 * @return voltage in volts
	 */
	protected abstract double getDriveVoltageInternal();

	/**
	 * Gets the voltage being used to power the steer motor.
	 * 
	 * @return voltage in volts
	 */
	protected abstract double getSteerVoltageInternal();

	/**
	 * Sets the voltage being used to power the drive motor.
	 * 
	 * @param voltage voltage in volts
	 */
	protected abstract void setDriveVoltageInternal(double voltage);

	/**
	 * Sets the voltage being used to power the steer motor.
	 * 
	 * @param voltage voltage in volts
	 */
	protected abstract void setSteerVoltageInternal(double voltage);

	/**
	 * Gets the voltage being used to power the drive motor regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @return voltage in volts
	 */
	public double getDriveVoltage() {
		return switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> getDriveVoltageInternal();
			case kSimulation -> m_driveVoltage;
		};
	}

	/**
	 * Gets the voltage being used to power the steer motor regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @return voltage in volts
	 */
	public double getSteerVoltage() {
		return switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> getSteerVoltageInternal();
			case kSimulation -> m_steerVoltage;
		};
	}

	/**
	 * Sets the voltage being used to power the drive motor regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @param voltage voltage in volts
	 */
	public void setDriveVoltage(double voltage) {
		switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> setDriveVoltageInternal(voltage);
			case kSimulation -> m_driveVoltage = voltage;
		}
	}

	/**
	 * Sets the voltage being used to power the steer motor regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @param voltage voltage in volts
	 */
	public void setSteerVoltage(double voltage) {
		switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> setSteerVoltageInternal(voltage);
			case kSimulation -> m_steerVoltage = voltage;
		}
	}

	/**
	 * Gets the number of rotations made by the drive wheel regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @return distance in rotations
	 */
	public double getDriveRotations() {
		return switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> getDriveRotationsInternal();
			case kSimulation -> m_driveRotations;
		};
	};

	/**
	 * Sets the number of rotations made by the drive wheel regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @param rotations distance in rotations
	 */
	public void setDriveRotations(double rotations) {
		switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> setDriveRotationsInternal(rotations);
			case kSimulation -> m_driveRotations = rotations;
		}
	};

	/**
	 * Resets the number of rotations made by the drive wheel to zero.
	 */
	public void resetDriveRotations() {
		setDriveRotations(0);
	};
}
