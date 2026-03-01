package frc.robot.subsystems;

import frc.robot.Constants.Subsystems.TurretConstants;

public class Turret extends AngularPositionSubsystem {
	private static AngularPositionSubsystem s_theTurret;

	/**
	 * Call create() instead
	 */
	private Turret() {
		super(TurretConstants.kTurretPort, "Turret", TurretConstants.kP, TurretConstants.kI, TurretConstants.kCurrent,
				TurretConstants.kSmartCurrent, TurretConstants.kMinAngle, TurretConstants.kMaxAngle,
				TurretConstants.kMaxDutyCycle, true);
	}

	public static void create() {
		if (s_theTurret == null) {
			s_theTurret = new Turret();
		} else {
			throw new Error("Turret already instantiated");
		}
	}

	public static AngularPositionSubsystem getTurret() {
		return s_theTurret;
	}
}