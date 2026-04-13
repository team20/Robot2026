package frc.robot.subsystems;

import frc.robot.Constants.Subsystems.HoodConstants;
import frc.robot.ControlUtils.ClampedP;

public class Hood extends AngularPositionSubsystem {
	private static AngularPositionSubsystem s_theHood;

	/**
	 * Call create() instead
	 */
	private Hood() {
		super(HoodConstants.kHoodPort, "Hood", HoodConstants.kP, HoodConstants.kI, HoodConstants.kS,
				HoodConstants.kCurrent,
				HoodConstants.kSmartCurrent, HoodConstants.kMinAngle, HoodConstants.kMaxAngle,
				1, HoodConstants.kMaxDutyCycle, HoodConstants.kMotorInvert, HoodConstants.kEncoderInvert,
				HoodConstants.kTolerance);

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

	public static ClampedP.Constants getConstants() {
		return new ClampedP.Constants(HoodConstants.kMinPower, HoodConstants.kMaxPower, HoodConstants.kMaxErr,
				HoodConstants.kTolerance);
	}
}