package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.Subsystems.IntakeConstants;

public class IntakeArm extends PositionControlSubsystem {

	public static IntakeArm s_theIntakeArm;

	public IntakeArm() {
		super(IntakeConstants.kIntakeArmPort, true, IntakeConstants.kArmMaxPower);

		SparkMaxConfig config = new SparkMaxConfig();

		config.smartCurrentLimit(IntakeConstants.kArmSmartCurrentLimit);
		config.secondaryCurrentLimit(IntakeConstants.kArmSecondaryCurrentLimit);
		config.idleMode(IdleMode.kBrake);
		config.inverted(IntakeConstants.kArmInvert);
		// REVLib 2027 removed positionConversionFactor. The 2026 call here set it on the absolute
		// encoder, but this arm uses the primary encoder for both feedback and getMotorRotations(),
		// so it had no effect and was dropped. TODO: confirm the absolute encoder isn't meant to be
		// the feedback sensor (kArmConversionFactor is now unused).
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