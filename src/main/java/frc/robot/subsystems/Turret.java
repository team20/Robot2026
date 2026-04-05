package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

	public double getTicksToAngle() {
		return s_theTurret.getPosition() / TurretConstants.kAngleToTicksFactor;
	}

	public double getRobotRelativeAngle() {
		double fromCenterPosition = getPosition() - TurretConstants.kStraightAheadAngle;
		return fromCenterPosition / TurretConstants.kAngleToTicksFactor + 180;
	}

	public double getTurretToWorldAngleRotation(Pose2d robotPose, double angle) {
		double botAngle = angle + robotPose.getRotation().getDegrees() + 180;
		double botTicks = Turret.getAngleToTicks(botAngle);
		double botTicksCentered = (botTicks + TurretConstants.kStraightAheadAngle) % Turret.getAngleToTicks(360);

		while (botTicksCentered < 0) {
			botTicksCentered += Turret.getAngleToTicks(360);
		}

		SmartDashboard.putNumber("TurnToWorldAngle/worldAngle", angle);
		SmartDashboard.putNumber("TurnToWorldAngle/botAngle", botAngle);
		SmartDashboard.putNumber("TurnToWorldAngle/botTicks", botTicks);
		SmartDashboard.putNumber("TurnToWorldAngle/botTicksCentered", botTicksCentered);

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

	public static ClampedPConstants getConstants() {
		return new ClampedPConstants(TurretConstants.kMinPower, TurretConstants.kMaxPower, TurretConstants.kMaxErr,
				TurretConstants.kTolerance);
	}
}