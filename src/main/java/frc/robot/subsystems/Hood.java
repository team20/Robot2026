package frc.robot.subsystems;

import frc.robot.Constants.Subsystems.HoodConstants;

public class Hood extends AngularPositionSubsystem {
	private static AngularPositionSubsystem s_theHood;

	/**
	 * Call create() instead
	 */
	private Hood() {
		super(HoodConstants.kHoodPort, "Hood", HoodConstants.kP, HoodConstants.kI, HoodConstants.kCurrent,
				HoodConstants.kSmartCurrent, HoodConstants.kMinAngle, HoodConstants.kMaxAngle,
				HoodConstants.kMaxDutyCycle, true);
	}

	public static void create() {
		if (s_theHood == null) {
			s_theHood = new Hood();
		} else {
			throw new Error("Hood already instantiated");
		}
	}

	public static AngularPositionSubsystem getHood() {
		return s_theHood;
	}
}