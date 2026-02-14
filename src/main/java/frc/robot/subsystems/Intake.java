package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.commands.IntakeCommands;

public class Intake extends SubsystemBase {
	private static Intake s_theIntake;
	private final SparkMax m_intakeWheels;
	private final SparkMaxConfig m_wheelConfig;
	private final SparkMax m_intakeArm;
	private final SparkMaxConfig m_armConfig;

	// Set up motor & initialize other subsystem aspects
	public Intake() {
		// Instantiate motor & motor configurations
		m_intakeWheels = new SparkMax(IntakeConstants.kIntakeWheelsPort, MotorType.kBrushless);
		m_wheelConfig = new SparkMaxConfig();

		// TODO: Check configuration of motors
		m_wheelConfig.smartCurrentLimit(IntakeConstants.kWheelSmartCurrentLimit);
		m_wheelConfig.secondaryCurrentLimit(IntakeConstants.kWheelSecondaryCurrentLimit);
		m_wheelConfig.idleMode(IdleMode.kBrake);
		m_wheelConfig.inverted(IntakeConstants.kWheelInvert);

		m_intakeWheels.configure(m_wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		m_intakeWheels.set(0.5);
		m_intakeArm = new SparkMax(IntakeConstants.kIntakeArmPort, MotorType.kBrushless);
		m_intakeArm.getEncoder().setPosition(0);
		m_armConfig = new SparkMaxConfig();

		// TODO: Check configuration of motors
		m_armConfig.smartCurrentLimit(IntakeConstants.kArmSmartCurrentLimit);
		m_armConfig.secondaryCurrentLimit(IntakeConstants.kArmSecondaryCurrentLimit);
		m_armConfig.idleMode(IdleMode.kBrake);
		m_armConfig.inverted(IntakeConstants.kArmInvert);
		m_armConfig.encoder.positionConversionFactor(IntakeConstants.kArmConversionFactor);

		m_intakeArm.configure(m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		if (s_theIntake == null) {
			s_theIntake = this;
		} else {
			throw new Error("Intake already instantiated");
		}
	}

	public static Intake getIntake() {
		return s_theIntake;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Intake/Arm Position", getArmRotations());
	}

	/**
	 * Set power of motor driving intake roller wheels.
	 * 
	 * @param speed
	 */
	public static void setWheelPower(double power) {
		s_theIntake.m_intakeWheels.set(power);
	}

	public static void setArmPower(double power) {
		s_theIntake.m_intakeArm.set(power);
	}

	public static double getArmRotations() {
		return s_theIntake.m_intakeArm.getEncoder().getPosition();
	}

	public static void stopWheel() {
		s_theIntake.m_intakeWheels.stopMotor();
	}

	public static void stopArm() {
		s_theIntake.m_intakeArm.stopMotor();
	}

	public static boolean isReverseLimitActive() {
		return s_theIntake.m_intakeArm.getReverseLimitSwitch().isPressed();
	}

	public static void resetArmEncoder() {
		s_theIntake.m_intakeArm.getEncoder().setPosition(0);
	}

	public static Command getResetCommand() {
		return new SequentialCommandGroup(
				new IntakeCommands.SpinArmPowerForTime(.4, 5),
				new IntakeCommands.ResetEncoder());
	}

	public static Command getExtendCommand() {
		return new IntakeCommands.MoveArmToPosition(IntakeConstants.kArmDeployRotations);
	}

	public static Command getRetractCommand() {
		return new IntakeCommands.MoveArmToPosition(IntakeConstants.kArmRetractRotations);
	}
}