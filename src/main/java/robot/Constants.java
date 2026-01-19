package robot;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import robot.utilities.BotConstants;
import robot.utilities.Compliance.FixMe;

public class Constants {
	public static final class Subsystems {
		public static final class TurretConstants {
			public static final double kDeadzone = .05;

			public static final class Yaw {
				@FixMe(reason = "Talk to electrical")
				public static final int kMotorPort = -1;
				@FixMe(reason = "Tune value")
				public static final double kP = 0;
			}

			public static final class Pitch {
				@FixMe(reason = "Talk to electrical")
				public static final int kMotorPort = -1;
				@FixMe(reason = "Tune value")
				public static final double kP = 0;
			}
		}

		public static final class TransportConstants {
			@FixMe(reason = "Talk to electrical")
			public static final int kMotorPort = -1;
			@FixMe(reason = "Determine speed")
			public static final double kDefaultSpeed = 0;
		}

		public static final class ShooterConstants {
			@FixMe(reason = "Talk to electrical")
			public static final int kMotorPort = -1;
			@FixMe(reason = "Determine speed")
			public static final double kDefaultSpeed = 0;
		}

		public static final class ClimberConstants {
			@FixMe(reason = "Talk to electrical")
			public static final int kMotorPort = -1;
			@FixMe(reason = "Determine speed")
			public static final double kDefaultSpeed = 0;
		}

		public static final class IntakeConstants {
			@FixMe(reason = "Talk to electrical")
			public static final int kRollerPort = -1;
			@FixMe(reason = "Talk to electrical")
			public static final int kDeployPort = -1;
			@FixMe(reason = "Determine speed")
			public static final double kDefaultSpeed = 0;
			@FixMe(reason = "Determine angle")
			public static final double kRetractAngle = 0;
			@FixMe(reason = "Determine angle")
			public static final double kDeployAngle = 0;
			@FixMe(reason = "Tune value")
			public static final double kP = 0;
			@FixMe(reason = "Determine velocity")
			public static final double kStationaryVelocity = 0;
		}
	}

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.05;
		public static final double kTriggerDeadzone = .05;
	}

	public static final class DriveConstants {
		// CAN IDs (updated)
		public static final int kFrontRightDrivePort = 10;
		public static final int kFrontRightSteerPort = 11;
		public static final int kFrontLeftDrivePort = 40;
		public static final int kFrontLeftSteerPort = 41;
		public static final int kBackRightDrivePort = 20;
		public static final int kBackRightSteerPort = 21;
		public static final int kBackLeftDrivePort = 30;
		public static final int kBackLeftSteerPort = 31;
		public static final int kFrontRightCANCoderPort = 12;
		public static final int kFrontLeftCANCoderPort = 42;
		public static final int kBackRightCANCoderPort = 22;
		public static final int kBackLeftCANCoderPort = 32;

		// TODO: Make sure these are tuned (can do with SysId)
		public static final double kP = 0.09;
		public static final double kI = 0.0;
		public static final double kD = 0.001;
		public static final double kS = 0;
		public static final double kV = 0.12;
		public static final double kA = 0.009;

		public static final double kRotationP = 5; // TODO: tune it
		public static final double kRotationI = 0.0;
		public static final double kRotationD = 0.1; // TODO: tune it
		public static final double kRotationS = 0;
		public static final double kRotationV = 1.9;
		public static final double kRotationA = 0.009;

		public static final Map<PhysicalBot, BotConstants> kBots = Map.ofEntries(
				new BotConstants(PhysicalBot.COMP_BOT)
						.setTeleopMaxVoltage(12)
						.setTeleopMaxTurnVoltage(7.2)
						.setDriveGearRatio(6.75)
						.setSteerGearRatio(150. / 7)
						.setWheelDiameter(4),
				new BotConstants(PhysicalBot.PRAC_BOT)
						.setTeleopMaxVoltage(12)
						.setTeleopMaxTurnVoltage(7.2)
						.setDriveGearRatio(6.75)
						.setSteerGearRatio(150. / 7)
						.setWheelDiameter(4),
				new BotConstants(PhysicalBot.DENNIS)
						.setTeleopMaxVoltage(12)
						.setTeleopMaxTurnVoltage(7.2)
						.setDriveGearRatio(6.75)
						.setSteerGearRatio(150. / 7)
						.setWheelDiameter(4));

		// https://docs.wpilib.org/en/latest/docs/software/basic-programming/coordinate-system.html
		public static final double kModuleDistFromCenter = Units.inchesToMeters(14.5); // Width/2
		public static final Translation2d kFrontLeftLocation = new Translation2d(kModuleDistFromCenter,
				kModuleDistFromCenter);
		public static final Translation2d kFrontRightLocation = new Translation2d(kModuleDistFromCenter,
				-kModuleDistFromCenter);
		public static final Translation2d kBackLeftLocation = new Translation2d(-kModuleDistFromCenter,
				kModuleDistFromCenter);
		public static final Translation2d kBackRightLocation = new Translation2d(-kModuleDistFromCenter,
				-kModuleDistFromCenter);

		public static final int kEncoderDepth = 4;
		public static final int kEncoderMeasurementPeriod = 16;
		// The amount of time to go from 0 to full power in seconds
		public static final double kRampRate = .1;
		public static final TalonFXConfiguration kDriveConfig = new TalonFXConfiguration();
		static {
			kDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			kDriveConfig.CurrentLimits.SupplyCurrentLimit = 45; // For avoiding brownout
			kDriveConfig.CurrentLimits.SupplyCurrentLowerLimit = 45;
			kDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			kDriveConfig.CurrentLimits.StatorCurrentLimit = 80; // Output current (proportional to acceleration)
			kDriveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			kDriveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampRate;
			kDriveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = kRampRate;
		}

		public static final TalonFXConfiguration kSteerConfig = new TalonFXConfiguration();
		static {
			kSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			kSteerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = kRampRate;
			kSteerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampRate;
			kSteerConfig.CurrentLimits.StatorCurrentLimit = 60;
			kSteerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			kSteerConfig.CurrentLimits.SupplyCurrentLimit = 75;
			kSteerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		}

		public static final double kTeleopDriveMaxSpeed = 12.0; // 5 meters per second
		public static final double kTeleopTurnMaxAngularSpeed = Math.toRadians(360 * 5);

		public static final double kDriveMaxSpeed = 12.0; // 5 meters per second
		public static final double kDriveMinSpeed = 0.2; // 0.2 meters per second
		public static final double kTurnMaxAngularSpeed = Math.toRadians(360); // 1 rotation per second
		public static final double kTurnMinAngularSpeed = Math.toRadians(0); // 0 degree per second

		// DriveCommand.java Constants
		public static final double kDriveP = 7;
		public static final double kDriveI = 0;
		public static final double kDriveD = 0;
		public static final double kDriveMaxAcceleration = 2 * kDriveMaxSpeed; // kDriveMaxSpeed in 1.5 sec

		public static final double kTurnP = 5;
		public static final double kTurnI = 0;
		public static final double kTurnD = 0.1;
		public static final double kTurnMaxAcceleration = 2 * kTurnMaxAngularSpeed; // kTurnMaxAngularSpeed in 0.5
	}
}
