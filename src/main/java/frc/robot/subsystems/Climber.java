package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.Subsystems.ClimberConstants;
import frc.robot.Constants.Subsystems.IntakeConstants;

public class Climber extends PositionControlSubsystem {

	public static Climber s_theClimber;

	public Climber() {
		super(ClimberConstants.kClimberPort);

		SparkMaxConfig config = new SparkMaxConfig();

		config.smartCurrentLimit(ClimberConstants.kSmartCurrentLimit);
		config.secondaryCurrentLimit(ClimberConstants.kSecondaryCurrentLimit);
		config.idleMode(IdleMode.kBrake);
		config.inverted(ClimberConstants.kInvert);
		config.encoder.positionConversionFactor(ClimberConstants.kGearRatio);
		config.closedLoop.pid(IntakeConstants.kP, 0, 0);
		config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
		config.closedLoop.positionWrappingEnabled(false);

		configureMotor(config);

		if (s_theClimber == null) {
			s_theClimber = this;
		} else {
			throw new Error("Intake Arm subsystem is already constructed.");
		}
	}

	public static Climber getClimber() {
		return s_theClimber;
	}
}