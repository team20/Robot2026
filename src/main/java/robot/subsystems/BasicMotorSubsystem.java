package robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An abstract subsystem which contains a single motor that can be either moved
 * for a specific duration or using a controller button.
 * 
 * See {@link Transport}, {@link Intake}, {@link Shooter}, or {@link Climber}
 * for examples of how to use this class.
 */
public abstract class BasicMotorSubsystem extends SubsystemBase {
	/**
	 * The motor controller allows us to change the speed of the motor
	 */
	private final SparkMax m_motor;
	/**
	 * The default speed of the motor is stored for use in commands
	 */
	private final double m_speed;

	/**
	 * Creates a new subsystem using the default speed and CAN id provided by the
	 * implementing class.
	 */
	public BasicMotorSubsystem() {
		m_motor = new SparkMax(getMotorId(), MotorType.kBrushless);
		m_speed = getDefaultSpeed();
	}

	/**
	 * Gets the CAN id for the motor controller.
	 *
	 * @return CAN id
	 */
	protected abstract int getMotorId();

	/**
	 * Gets the default speed for the motor.
	 *
	 * @return default speed
	 */
	protected abstract double getDefaultSpeed();

	/**
	 * Creates a new command that moves the motor at a fixed speed for the specified
	 * duration.
	 *
	 * @param speed motor speed
	 * @param time duration in seconds
	 * @return command
	 */
	public Command moveForTime(double speed, double time) {
		return startEnd(
				() -> m_motor.set(speed), // Set the motor speed to the specified value
				() -> m_motor.stopMotor()) // Stop the motor when the command ends
						.withTimeout(time) // End the command after the specified duration
						.withName("moveForTime");
	}

	/**
	 * Creates a new command that moves the motor at the default speed for the
	 * specified duration.
	 *
	 * @param time duration in seconds
	 * @return command
	 */
	public Command moveForTime(double time) {
		return moveForTime(m_speed, time);
	}

	/**
	 * Creates a new command that moves the motor at a fixed speed when buttons are
	 * pressed.
	 *
	 * @param speed motor speed
	 * @param forward button which causes forward movement
	 * @param back button which causes backward movement
	 * @return command
	 */
	public Command moveWithTrigger(double speed, Trigger forward, Trigger back) {
		return runEnd(
				() -> {
					double sum = 0; // Don't move the motor if no triggers are active.
					if (forward != null) {
						if (forward.getAsBoolean()) {
							sum += speed; // Increase the speed when the forward trigger is active.
						}
					}
					if (back != null) {
						if (back.getAsBoolean()) {
							sum -= speed; // Decrease the speed when the backward trigger is active.
						}
					}
					m_motor.set(sum); // Set the motor speed based on the active triggers.
				},
				() -> m_motor.stopMotor()) // Stop the motor when the command ends
						.withName("moveWithTrigger");
	}

	/**
	 * Creates a new command that moves the motor at the default speed when buttons
	 * are pressed.
	 *
	 * @param forward button which causes forward movement
	 * @param back button which causes backward movement
	 * @return command
	 */
	public Command moveWithTrigger(Trigger forward, Trigger back) {
		return moveWithTrigger(m_speed, forward, back);
	}
}
