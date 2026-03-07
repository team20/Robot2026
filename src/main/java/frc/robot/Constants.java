package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
	public static final boolean kCompBot = false;
	public static final boolean kLogging = true;

	public static final class Subsystems {
		public static final class TurretConstants {
			public static final int kTurretPort = 50;
			public static final double kMaxDutyCycle = 0.5;
			public static final double kMinPower = 0.025;
			public static final double kMaxPower = 0.25;
			public static final double kMaxErr = 25;
			public static final double kTolerance = 3;
			public static final double kP = 0.006;
			public static final double kI = 0.0000;
			public static final double kLargeDeadzone = 0.5; // For the X/Y joystick control
			public static final double kSmallDeadzone = 0.05;
			public static final int kSmartCurrent = 20;
			public static final int kCurrent = 25;
			public static final double kMinAngle = 50;
			public static final double kMaxAngle = 270;
		}

		public static final class HoodConstants {
			public static final int kHoodPort = 52;
			public static final double kMaxDutyCycle = 0.5;
			public static final double kMinPower = 0.025;
			public static final double kMaxPower = 0.25;
			public static final double kMaxErr = 25;
			public static final double kTolerance = 1;
			public static final double kP = 0.01;
			public static final double kI = 0.0000;
			public static final double kDeadzone = 0.05;
			public static final int kSmartCurrent = 20;
			public static final int kCurrent = 25;
			public static final double kMinAngle = 0;
			public static final double kMaxAngle = 38;
		}

		public static final class ShooterConstants {
			public static final int kFlywheelPort = 51;
			public static final double kCurrentLimit = 30;
			public static final double kV = 480;
			public static final int kDefaultDistance = 10;
			public static final double kRampRate = 10;
		}

		public static final class IntakeConstants {
			public static final int kIntakeWheelsPort = 54;
			public static final int kIntakeArmPort = 53;
			public static final double kArmConversionFactor = 0.01;

			public static final int kWheelSmartCurrentLimit = 10;
			public static final int kWheelSecondaryCurrentLimit = 20;
			public static final boolean kWheelInvert = false;

			public static final int kArmSmartCurrentLimit = 10;
			public static final int kArmSecondaryCurrentLimit = 20;
			public static final boolean kArmInvert = false;

			public static final double kArmPower = 0.5;
			public static final double kArmRetractRotations = 170;
			public static final double kArmDeployRotations = 0;

			public static final double kOutPosition = 8; // TODO: Find actual positions
			public static final double kInPosition = 0;
			public static final double kP = 0.2;
		}

		public static final class TransportConstants {
			public static final int kKickerPort = 56;
			public static final int kKickerSmartCurrentLimit = 10;
			public static final int kKickerSecondaryCurrentLimit = 20;
			public static final int kAgitatorPort = 57;
			public static final int kAgitatorSmartCurrentLimit = 20;
			public static final int kAgitatorSecondaryCurrentLimit = 30;
		}

		public static final class ClimberConstants {
			public static final int kClimberPort = 55;
			public static final int kSmartCurrentLimit = 30;
			public static final int kSecondaryCurrentLimit = 40;
			public static final boolean kInvert = false;
			public static final double kGearRatio = 81.0 / 1.0;
			public static final double kClimbPosition = 27744; // TODO: Find actual positions
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

		public static final double kDriveGearRatio = 6.75;
		public static final double kSteerGearRatio = 150.0 / 7; // TODO: Change value for 5i's
		public static final double kWheelDiameter = Units.inchesToMeters(4);
		public static final double kWheelCircumference = Math.PI * kWheelDiameter;

		public static final double kMetersPerMotorRotation = kWheelCircumference / kDriveGearRatio;

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
