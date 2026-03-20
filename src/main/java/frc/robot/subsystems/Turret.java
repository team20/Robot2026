package frc.robot.subsystems;

import frc.robot.ClampedP.ClampedPConstants;
import frc.robot.Constants.Subsystems.TurretConstants;

public class Turret extends AngularPositionSubsystem {
	private static Turret s_theTurret;

	/**
	 * Call create() instead
	 */
	private Turret() {
		super(TurretConstants.Motor.kTurretPort, "Turret", TurretConstants.Control.kP, TurretConstants.Control.kI,
				TurretConstants.Motor.kCurrent,
				TurretConstants.Motor.kSmartCurrent, TurretConstants.Geometry.kMinAngle,
				TurretConstants.Geometry.kMaxAngle,
				TurretConstants.Motor.kMaxDutyCycle, TurretConstants.Motor.kMotorInvert,
				TurretConstants.Motor.kEncoderInvert);
	}

	public static void create() {
		if (s_theTurret == null) {
			s_theTurret = new Turret();
		} else {
			throw new Error("Turret already instantiated");
		}
	}

	public static Turret getTurret() {
		return s_theTurret;
	}

	public static ClampedPConstants getConstants() {
		return new ClampedPConstants(TurretConstants.Motor.kMinPower, TurretConstants.Control.kMaxPower,
				TurretConstants.Control.kMaxErr,
				TurretConstants.Control.kTolerance);
	}
}