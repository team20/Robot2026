package frc.robot.subsystems;

import frc.robot.ClampedP.ClampedPConstants;
import frc.robot.Constants.Subsystems.TurretConstants;

public class Turret extends AngularPositionSubsystem {
	private static Turret s_theTurret;

	/**
	 * Call create() instead
	 */
	private Turret() {
		super(TurretConstants.kTurretPort, "Turret", TurretConstants.kP, TurretConstants.kI, TurretConstants.kS,
				TurretConstants.kCurrent,
				TurretConstants.kSmartCurrent, TurretConstants.kMinAngle, TurretConstants.kMaxAngle,
				TurretConstants.kPositionConversionFactor, TurretConstants.kMaxDutyCycle, TurretConstants.kMotorInvert,
				TurretConstants.kEncoderInvert, TurretConstants.kTolerance);
	}

	public static double getAngleToTicks(double realAngle) {
		return realAngle * TurretConstants.kAngleToTicksFactor;
	}

	public double getRobotRelativeAngle() {
		double fromCenterPosition = getPosition() - TurretConstants.kStraightAheadAngle;
		return fromCenterPosition / TurretConstants.kAngleToTicksFactor + 180;
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
		return new ClampedPConstants(TurretConstants.kMinPower, TurretConstants.kMaxPower, TurretConstants.kMaxErr,
				TurretConstants.kTolerance);
	}
}