package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
//import frc.robot.Compliance.FixMe;

public class Constants {
	public static final boolean kLogging = true;

	public static final class Subsystems {
		public static final class TurretConstants {
			public static final double kMaxDutyCycle = 0.5;
			public static final double kMinPower = 0.025;
			public static final double kMaxPower = 0.25;
			public static final double kMaxErr = 25;
			public static final double kTolerance = 3;
			public static final double kP = 0.006;
			public static final double kI = 0.0000;
			public static final double kLargeDeadzone = 0.5; // For the X/Y joystick control
			public static final double kSmallDeadzone = 0.05;
		}

		public static final class HoodConstants {
			public static final double kMaxDutyCycle = 0.5;
			public static final double kMinPower = 0.025;
			public static final double kMaxPower = 0.25;
			public static final double kMaxErr = 25;
			public static final double kTolerance = 1;
			public static final double kP = 0.01;
			public static final double kDeadzone = 0.05;
		}

		public static final class ShooterConstants {
			public static final int kMotorPort = 23;
			public static final double kCurrentLimit = 30;
			public static final double kV = 480;
			public static final int kDefaultRPM = 2400; // TODO: Test and find actual default RPM
			public static final double kRampRate = 500;
		}

		public static final class IntakeConstants {
			public static final int kIntakeWheelsPort = 2; // TODO: Update CAN IDs, these are placeholder values
			public static final int kIntakeArmPort = 1;
			public static final int kWheelSmartCurrentLimit = 10;
			public static final int kWheelSecondaryCurrentLimit = 20;
			public static final boolean kWheelInvert = false;
			public static final int kArmSmartCurrentLimit = 10;
			public static final int kArmSecondaryCurrentLimit = 20;
			public static final boolean kArmInvert = false;
			public static final double kArmPower = 0.5;
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

		public static final double kTeleopMaxVoltage = 12;
		public static final double kTeleopMaxTurnVoltage = 7.2;
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