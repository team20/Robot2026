package robot.subsystems;

import robot.Constants.Subsystems.ClimberConstants;
import robot.utilities.Compliance;

/**
 * A subystem which moves in two directions to allow the robot to both climb and
 * declimb.
 */
public class Climber extends BasicMotorSubsystem {
	/**
	 * Creates a new subsystem with a proper name
	 */
	public Climber() {
		super();
		setName("Climber subsystem");
	}

	/**
	 * Gets the CAN id from the climber constants for the motor controller.
	 *
	 * @return CAN id
	 */
	@Override
	protected int getMotorId() {
		return Compliance.ensure(ClimberConstants.class, "kMotorPort");
	}

	/**
	 * Gets the default speed from the climber constants for the motor.
	 *
	 * @return default speed
	 */
	@Override
	protected double getDefaultSpeed() {
		return Compliance.ensure(ClimberConstants.class, "kDefaultSpeed");
	}

}
