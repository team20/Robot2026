package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.Subsystems.IntakeConstants;

public class IntakeArm extends PositionControlSubsystem {

	public static IntakeArm s_theIntakeArm;

	public IntakeArm() {
		super(IntakeConstants.kIntakeArmPort);

		SparkMaxConfig config = new SparkMaxConfig();

		config.smartCurrentLimit(IntakeConstants.kArmSmartCurrentLimit);
		config.secondaryCurrentLimit(IntakeConstants.kArmSecondaryCurrentLimit);
		config.idleMode(IdleMode.kBrake);
		config.inverted(IntakeConstants.kArmInvert);
		config.absoluteEncoder.positionConversionFactor(IntakeConstants.kArmConversionFactor);
		config.closedLoop.pid(IntakeConstants.kP, 0, 0);
		config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
		config.closedLoop.positionWrappingEnabled(false);

		configureMotor(config);

		if (s_theIntakeArm == null) {
			s_theIntakeArm = this;
		} else {
			throw new Error("Intake Arm subsystem is already constructed.");
		}
	}

	public static IntakeArm getIntakeArm() {
		return s_theIntakeArm;
	}
}