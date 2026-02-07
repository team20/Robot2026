package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
// import motor stuff
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem {
	private SparkMax m_motor;

	// initialize stuff
	@SuppressWarnings("deprecation")
	public IntakeSubsystem() {
		// can id is a placeholder; replace later.
		m_motor = new SparkMax(10, MotorType.kBrushless);
		// class SparkBaseConfig smartcurrentlimit(int stalllimit)
		// set current limits
		m_motor.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);
		m_motor.setInverted(IntakeConstants.kInvert);

	}

	// set speed for motor
	public void setSpeed(double speed) {
		m_motor.set(speed);
	}
}
