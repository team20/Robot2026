package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Subsystems.TransportConstants;

/**
 * The Transport subsystem is responsible for moving game pieces from the intake
 * to the shooter. It uses a motor controller to achieve this functionality.
 */
public class Transport extends BasicMotorSubsystem {

	private final PWMSparkMax motor;

	/**
	 * Constructs a new Transport subsystem.
	 */
	public Transport() {
		motor = new PWMSparkMax(TransportConstants.kMotorPort);
	}

	/**
	 * Sets the motor speed for the transport subsystem.
	 *
	 * @param speed The speed to set, between -1.0 and 1.0.
	 */
	public void setSpeed(double speed) {
		motor.set(speed);
	}

	/**
	 * Stops the transport motor.
	 */
	public void stop() {
		motor.set(0);
	}

	/**
	 * Creates a command to move the transport motor based on trigger inputs.
	 *
	 * @param triangle Trigger to move forward.
	 * @param cross Trigger to move backward.
	 * @return A command to control the motor.
	 */
	public Command moveWithTrigger(Trigger triangle, Trigger cross) {
		return run(() -> {
			if (triangle.getAsBoolean()) {
				motor.set(getDefaultSpeed());
			} else if (cross.getAsBoolean()) {
				motor.set(-getDefaultSpeed());
			} else {
				motor.set(0); // Default case to stop the motor
			}
		});
	}

	@Override
	protected int getMotorId() {
		return TransportConstants.kMotorPort;
	}

	@Override
	protected double getDefaultSpeed() {
		return TransportConstants.kDefaultSpeed;
	}
}
