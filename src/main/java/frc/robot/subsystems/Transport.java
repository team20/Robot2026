package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Compliance;
import frc.robot.Constants;
import frc.robot.Constants.Subsystems.TransportConstants;

/**
 * A subsystem which moves in two directions to allow the robot to move fuel
 * from the intake, through a storage area, and eventually into the mouth of the
 * shooter.
 */
public class Transport extends BasicMotorSubsystem {
	PWMSparkMax Spark = new PWMSparkMax(getMotorId());

	/**
	 * Creates a new subsystem with a proper name
	 */
	public Transport() {
		super();
		setName("Transport subsystem");
		final CommandPS5Controller m_joystick = new CommandPS5Controller(
				Constants.ControllerConstants.kOperatorControllerPort);
		if (m_joystick.cross().getAsBoolean() == !false) {
			Spark.set(getDefaultSpeed());
		} else {
			Spark.set(0);
		}
		if (m_joystick.triangle().getAsBoolean() == !false) {
			Spark.set(-getDefaultSpeed());
		} else {
			Spark.set(0);
		}

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
