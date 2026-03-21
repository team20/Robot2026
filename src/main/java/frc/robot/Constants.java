package frc.robot;

import java.util.Set;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
	public static final boolean kCompBot = true;
	public static final boolean kLogging = true;

	public static final class Subsystems {
		public static final class VisionConstants {
			// 3 red, 3 blue
			public static final Set<Integer> kTrackableTags = Set.of(8, 24, 10, 26, 11, 27);
			public static final Pose2d kBlueHub = new Pose2d(4.63, 4.04, Rotation2d.kZero);
			public static final Pose2d kRedHub = new Pose2d(11.92, 4.04, Rotation2d.kZero);
		}

		public static final class TurretConstants {
			public static final class Motor {
				public static final boolean kMotorInvert = false; // Positive power must increase the angle
				public static final boolean kEncoderInvert = true;
				public static final int kTurretPort = 50;
				public static final double kMaxDutyCycle = 1; // extra limit for test safety
				public static final double kMinPower = 0.025;
				public static final int kSmartCurrent = 25;
				public static final int kCurrent = 30;
			}

			public static final class Control {
				public static final double kMaxPower = Motor.kMaxDutyCycle;
				public static final double kMaxErr = 25;
				public static final double kTolerance = 1.5;
				public static final double kP = 0.012;
				public static final double kI = 0.0000;
				public static final double kLargeDeadzone = 0.5; // For the X/Y joystick control
				public static final double kSmallDeadzone = 0.05;
			}

			public static final class Feedforward {
				public static final double kFrictionPower = 0.05; // TODO: Actually determine the real number
				public static final double kRPMPerPower = 11000; // TODO: Actually determine the real number
				public static final double kGearRatio = 42; // TODO: Actually determine the real number
				public static final double kRPMDeadzone = 1; // TODO: Actually determine the real number
			}

			public static final class Geometry {
				public static final double kMinAngle = 22; // B
				public static final double kMaxAngle = 183; // A // <-- 190 <-- 220
				public static final double kPositionConversionFactor = 7.0 / 6; // 7.0 / 6
				public static final double kStraightAheadAngle = 90; // 105
				public static final double kCameraOffsetFromTurret = Units.inchesToMeters(7.685);
				public static final double kTurretOffsetFromFrame = Units.inchesToMeters(5.004);
				public static final double kTurretHeightFromFloor = Units.inchesToMeters(20.125);
			}
		}

		public static final class HoodConstants {
			public static final boolean kMotorInvert = true;
			public static final boolean kEncoderInvert = false;
			public static final int kHoodPort = 52;
			public static final double kMaxDutyCycle = 0.5;
			public static final double kMinPower = 0.025;
			public static final double kMaxPower = 0.25;
			public static final double kMaxErr = 25;
			public static final double kTolerance = 1;
			public static final double kP = 0.04;
			public static final double kI = 0.0000;
			public static final double kDeadzone = 0.05;
			public static final int kSmartCurrent = 20;
			public static final int kCurrent = 25;
			public static final double kMinAngle = 99;
			public static final double kMaxAngle = 137;
		}

		public static final class ShooterConstants {
			public static final int kFlywheelPort = 51;
			public static final double kCurrentLimit = 60;
			public static final double kV = 480;
			public static final int kDefaultDistance = 10;
			public static final int kDefaultRPM = 2000;
			public static final double kRampRate = 10;
			public static final double kRPMRamp = 250;
		}

		public static final class IntakeConstants {
			public static final int kIntakeWheelsPort = 54;
			public static final int kIntakeArmPort = 53;
			public static final double kArmConversionFactor = 0.01;

			public static final int kWheelSmartCurrentLimit = 20;
			public static final int kWheelSecondaryCurrentLimit = 30;
			public static final boolean kWheelInvert = true;

			public static final int kArmSmartCurrentLimit = 10;
			public static final int kArmSecondaryCurrentLimit = 20;
			public static final boolean kArmInvert = true;

			public static final double kArmPower = 0.5;

			public static final double kOutPosition = 145;
			public static final double kInPosition = 10;
			public static final double kP = 0.1;
			public static final double kWheelPower = 1;
		}

		public static final class KickerConstants {
			public static final double kTeleopPower = .4;
			public static final int kKickerPort = 56;
			public static final int kKickerSmartCurrentLimit = 20;
			public static final int kKickerSecondaryCurrentLimit = 30;
			public static final boolean kKickerInvert = true;
		}

		public static final class AgitatorConstants {
			public static final double kTeleopPower = .75;
			public static final int kAgitatorPort = 57;
			public static final int kAgitatorSmartCurrentLimit = 40;
			public static final int kAgitatorSecondaryCurrentLimit = 50;
			public static final boolean kAgitatorInvert = true;
		}

		public static final class ClimberConstants {
			public static final int kClimberPort = 55;
			public static final int kSmartCurrentLimit = 40;
			public static final int kSecondaryCurrentLimit = 60;
			public static final boolean kInvert = false;
			public static final double kGearRatio = 81.0 / 1.0;
			public static final double kClimbPosition = 29000; // TODO: Find actual positions
			public static final double kRetractPosition = 0;
			public static final double kP = 0.2;
		}
	}

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
	}

	public static final class DriveConstants {
		public static final double kDeadzone = 0.05;
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
		public static final double kP = 0.01;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kV = 0.12;
		public static final double kA = 0.009;

		public static final double kDriveGearRatio = 6.03;
		public static final double kSteerGearRatio = 26; // TODO: Change value for 5i's
		public static final double kWheelDiameter = Units.inchesToMeters(4);
		public static final double kWheelCircumference = Math.PI * kWheelDiameter;

		public static final double kMetersPerMotorRotation = kWheelCircumference / kDriveGearRatio;

		// https://docs.wpilib.org/en/latest/docs/software/basic-programming/coordinate-system.html
		public static final double kModuleDistFromCenter = Units.inchesToMeters(10.875); // Width/2
		public static final Translation2d kFrontLeftLocation = new Translation2d(kModuleDistFromCenter,
				kModuleDistFromCenter);
		public static final Translation2d kFrontRightLocation = new Translation2d(kModuleDistFromCenter,
				-kModuleDistFromCenter);
		public static final Translation2d kBackLeftLocation = new Translation2d(-kModuleDistFromCenter,
				kModuleDistFromCenter);
		public static final Translation2d kBackRightLocation = new Translation2d(-kModuleDistFromCenter,
				-kModuleDistFromCenter);

		public static final TalonFXConfiguration kDriveConfig = new TalonFXConfiguration();
		static {
			kDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			kDriveConfig.CurrentLimits.SupplyCurrentLimit = 45; // For avoiding brownout
			kDriveConfig.CurrentLimits.SupplyCurrentLowerLimit = 45;
			kDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			kDriveConfig.CurrentLimits.StatorCurrentLimit = 80; // Output current (proportional to acceleration)
			kDriveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			kDriveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		}

		public static final TalonFXConfiguration kSteerConfig = new TalonFXConfiguration();
		static {
			kSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			kSteerConfig.CurrentLimits.StatorCurrentLimit = 60;
			kSteerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			kSteerConfig.CurrentLimits.SupplyCurrentLimit = 75;
			kSteerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			kSteerConfig.MotorOutput.Inverted = (kCompBot) ? InvertedValue.CounterClockwise_Positive
					: InvertedValue.Clockwise_Positive;
		}
	}
}
