package robot.subsystems;

import robot.Constants.Subsystems.TransportConstants;
import robot.utilities.Compliance;

/**
 * A subsystem which moves in two directions to allow the robot to move fuel
 * from the intake, through a storage area, and eventually into the mouth of the
 * shooter.
 */
public class Transport extends BasicMotorSubsystem {
	/**
	 * Creates a new subsystem with a proper name
	 */
	public Transport() {
		super();
		setName("Transport subsystem");
	}

	/**
	 * Gets the CAN id from the transport constants for the motor controller.
	 *
	 * @return CAN id
	 */
	@Override
	protected int getMotorId() {
		return Compliance.ensure(TransportConstants.class, "kMotorPort");
	}

	/**
	 * Gets the default speed from the transport constants for the motor.
	 *
	 * @return default speed
	 */
	@Override
	protected double getDefaultSpeed() {
		return Compliance.ensure(TransportConstants.class, "kDefaultSpeed");
	}

}
