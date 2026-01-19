package robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import robot.Constants.Subsystems.IntakeConstants;
import robot.utilities.Compliance;

/**
 * A subsystem which has both rollers and a deployment mechanism in order to
 * collect fuel over or through the bumper.
 */
public class Intake extends BasicMotorSubsystem {
	/**
	 * The deploy motor is used to lower the intake towards the field.
	 */
	private final SparkMax m_deploy;
	/**
	 * The PID controller is used to move the deploy motor to a specific angle, such
	 * as a deployed angle or a retracted angle.
	 */
	private final PIDController m_pid;
	/**
	 * The relative encoder of the motor is used to check for the current angle.
	 * This means that the mechanism must be reset before each match.
	 */
	private final RelativeEncoder m_encoder;

	/**
	 * Creates a new subsystem using the deploy motor id from the intake constants,
	 * the pid constants from the intake constants, and a proper name.
	 */
	public Intake() {
		setName("Intake subsystem");
		int id = Compliance.ensure(IntakeConstants.class, "kDeployPort");
		m_deploy = new SparkMax(id, MotorType.kBrushless);
		m_encoder = m_deploy.getEncoder();
		double kP = Compliance.ensure(IntakeConstants.class, "kP");
		m_pid = new PIDController(kP, 0, 0);
	}

	/**
	 * Creates a new command which moves the deploy motor to the specified angle.
	 * 
	 * @param angle angle in degrees
	 * @return command
	 */
	private Command moveToAngleCommand(double angle) {
		double velocity = Compliance.ensure(IntakeConstants.class, "kStationaryVelocity");
		return runEnd(
				() -> m_deploy.set(
						m_pid.calculate( // Use the PID controller to calculate the speed.
								m_encoder.getPosition())), // Get the current angle of the mechanism.
				() -> m_deploy.stopMotor()) // Stop the motor when the command ends.
						.beforeStarting(
								() -> m_pid.setSetpoint(angle)) // Set the target angle when the command is initialized.
						.until(
								() -> m_pid.atSetpoint() && m_encoder.getVelocity() < velocity) // End the command when
																								// the target angle is
																								// reached and the
																								// mechanism is
																								// relatively
																								// stationary.
						.withName("moveToAngleCommand");
	}

	/**
	 * Creates a new command which moves the deploy motor to the deployed position.
	 * 
	 * @return command
	 */
	public Command deployRollers() {
		double angle = Compliance.ensure(IntakeConstants.class, "kDeployAngle");
		return moveToAngleCommand(angle);
	}

	/**
	 * Creates a new command which moves the deploy motor to the retracted position.
	 * 
	 * @return command
	 */
	public Command retractRollers() {
		double angle = Compliance.ensure(IntakeConstants.class, "kRetractAngle");
		return moveToAngleCommand(angle);
	}

	/**
	 * Gets the CAN id from the intake constants for the rollers' motor controller.
	 *
	 * @return CAN id
	 */
	@Override
	protected int getMotorId() {
		return Compliance.ensure(IntakeConstants.class, "kRollerPort");
	}

	/**
	 * Gets the default speed from the intake constants for rollers' the motor.
	 *
	 * @return default speed
	 */
	@Override
	protected double getDefaultSpeed() {
		return Compliance.ensure(IntakeConstants.class, "kDefaultSpeed");
	}
}
