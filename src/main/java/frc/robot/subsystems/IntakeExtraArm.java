package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.Subsystems.IntakeConstants;

public class IntakeExtraArm extends PositionControlSubsystem {

	public static IntakeExtraArm s_theExtraIntakeArm;

	public IntakeExtraArm() {
		super(IntakeConstants.kIntakeExtraArmPort, true, 0.5f);

		SparkMaxConfig config = new SparkMaxConfig();

		config.smartCurrentLimit(IntakeConstants.kArmSmartCurrentLimit);
		config.secondaryCurrentLimit(IntakeConstants.kArmSecondaryCurrentLimit);
		config.idleMode(IdleMode.kCoast);
		config.inverted(false);
		config.absoluteEncoder.positionConversionFactor(IntakeConstants.kArmConversionFactor);
		config.closedLoop.pid(IntakeConstants.kP, 0, 0);
		config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
		config.closedLoop.positionWrappingEnabled(false);

		configureMotor(config);

		if (s_theExtraIntakeArm == null) {
			s_theExtraIntakeArm = this;
		} else {
			throw new Error("Intake Extra Arm subsystem is already constructed.");
		}
	}

	public static IntakeExtraArm getExtraIntakeArm() {
		return s_theExtraIntakeArm;
	}
}