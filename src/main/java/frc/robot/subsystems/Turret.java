package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Subsystems.TurretConstants;
import frc.robot.ControlUtils.ClampedP;

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

	/**
	 * Converts angle (in degrees) to turret rotation (in ticks).
	 * 
	 * @param realAngle
	 * @return ticks
	 */
	public static double getAngleToTicks(double realAngle) {
		return realAngle * TurretConstants.kAngleToTicksFactor;
	}

	/**
	 * Converts current turret position (in ticks) to an angle (in degrees).
	 * 
	 * @return degree angle
	 */
	public double getTicksToAngle() {
		return s_theTurret.getPosition() / TurretConstants.kAngleToTicksFactor;
	}

	/**
	 * Converts current turret position (in ticks) to
	 * robot pose orientation (in degrees).
	 * 
	 * @return pose heading
	 */
	public double getRobotRelativeAngle() {
		double fromCenterPosition = getPosition() - TurretConstants.kStraightAheadAngle;
		return fromCenterPosition / TurretConstants.kAngleToTicksFactor + 180;
	}

	/**
	 * Converts world-space angle (in degrees) to turret position (in ticks).
	 * 
	 * @param robotPose
	 * @param angle (degrees)
	 * @return turret position (ticks)
	 */
	public double getTurretToWorldAngleRotation(Pose2d robotPose, double angle) {

		// Calculate robot pose heading from world-space angle
		// Convert robot pose heading to turret position (in ticks)
		double botAngle = angle + robotPose.getRotation().getDegrees() + 180;
		double botTicks = Turret.getAngleToTicks(botAngle);
		double botTicksCentered = (botTicks + TurretConstants.kStraightAheadAngle)
				// Wrap turret position
				% Turret.getAngleToTicks(360);

		// Rotate needed turret position until positive
		while (botTicksCentered < 0) {
			botTicksCentered += Turret.getAngleToTicks(360);
		}

		// Publish values
		SmartDashboard.putNumber("TurnToWorldAngle/worldAngle", angle);
		SmartDashboard.putNumber("TurnToWorldAngle/botAngle", botAngle);
		SmartDashboard.putNumber("TurnToWorldAngle/botTicks", botTicks);
		SmartDashboard.putNumber("TurnToWorldAngle/botTicksCentered", botTicksCentered);

		// Return needed turret position
		return botTicksCentered;
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

	public static ClampedP.Constants getConstants() {
		return new ClampedP.Constants(TurretConstants.kMinPower, TurretConstants.kMaxPower, TurretConstants.kMaxErr,
				TurretConstants.kTolerance);
	}
}